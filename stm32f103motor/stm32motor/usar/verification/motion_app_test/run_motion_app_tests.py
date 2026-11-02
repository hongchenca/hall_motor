"""Execute real motor_app.c UART commands and motor_motion.c in ARM Unicorn.

BLDC, encoder, CAN and HAL boundaries are mocked. No command parser or motion
algorithm is copied. Does not verify real serial hardware or mechanical motion.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import traceback

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent / "motion_test"))
import run_motion_tests as runtime
from unicorn import UC_HOOK_MEM_WRITE

ELF_PATH = HERE / "motion_app_mock.axf"
runtime.ELF_PATH = ELF_PATH
FIELDS = runtime.FIELDS + ["voltage", "period", "stops", "driver_fault", "now", "command_ms"]


def build():
    common = [str(runtime.ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M3", "--thumb",
              "--c99", "-O1", "-g", "-I", str(HERE / "mock"), "-I", str(PROJECT / "usar")]
    sources = [PROJECT / "usar" / "motor_app.c", PROJECT / "usar" / "motor_motion.c", HERE / "harness.c"]
    objects = [HERE / (p.stem + ".o") for p in sources]
    commands = [common + ["-c", str(p), "-o", str(o)] for p, o in zip(sources, objects)]
    commands.append([str(runtime.ARM_BIN / "armlink.exe"), "--cpu", "Cortex-M3", "--entry", "test_entry",
                     "--ro-base", "0x10000", "--rw-base", "0x20000000", "--no_remove", "--datacompressor=off",
                     *(str(p) for p in objects), "--output", str(ELF_PATH)])
    logs = []
    for command in commands:
        result = subprocess.run(command, capture_output=True, text=True, errors="replace", cwd=HERE)
        logs.append(subprocess.list2cmdline(command) + "\n" + result.stdout + result.stderr)
        (HERE / "build.log").write_text("\n".join(logs), encoding="utf-8")
        if result.returncode:
            raise RuntimeError(logs[-1])
    return commands


class App(runtime.Motion):
    def __init__(self):
        super().__init__(now=1000)
        self.tx = bytearray()
        self.block_txe_after_byte = False
        address = self.symbols["mock_usart3"] + 4

        def transmitted(uc, access, address, size, value, user_data):
            self.tx.append(value & 255)
            if self.block_txe_after_byte:
                uc.mem_write(self.symbols["mock_usart3"], struct.pack("<I", 0))

        self.uc.hook_add(UC_HOOK_MEM_WRITE, transmitted, begin=address, end=address + 3)
        self.drain()

    def snapshot(self):
        self.call("Test_Snapshot")
        return dict(zip(FIELDS, struct.unpack("<20i", self.uc.mem_read(self.symbols["test_snapshot"], 80))))

    def sample(self, flags=3, ready=1, fault=0, sample_ms=None):
        self.call("Test_Driver", ready, fault)
        self.call("Test_Encoder", round(self.raw), self.now if sample_ms is None else sample_ms, flags)
        self.call("Test_Run", self.now)
        return self.snapshot()

    def drain(self):
        for _ in range(128):
            length = len(self.tx)
            self.call("Test_Run", self.now)
            if len(self.tx) == length:
                return
        raise AssertionError("console transmit queue never drained")

    def command(self, text, drain=True):
        data = (text + "\r\n\0").encode("ascii")
        assert len(data) <= 256
        self.tx.clear()
        self.uc.mem_write(self.symbols["test_command"], data)
        self.call("Test_Command")
        self.now = self.snapshot()["now"] & 0xFFFFFFFF
        if drain:
            self.drain()
        assert struct.unpack("<I", self.uc.mem_read(self.symbols["blocking_transmit_calls"], 4))[0] == 0
        return self.tx.decode("ascii")

    def start(self, rpm=30, direction="FWD"):
        assert "OK" in self.command(f"MOTOR RPM {rpm}")
        assert "OK" in self.command(f"MOTOR DIR {direction}")
        assert "OK" in self.command("MOTOR START")
        self.advance(240, velocity=0)
        s = self.snapshot()
        assert s["active"] and s["voltage"] * (1 if direction == "FWD" else -1) > 0, s
        return s


def stopped(s):
    assert s["drive"] == 0 and s["voltage"] == 0, s


def rpm_start_and_legacy():
    a = App()
    assert "OK" in a.command("MOTOR RPM 30")
    s = a.snapshot()
    stopped(s)
    assert s["rpm_limit"] == 30 and not s["active"], s
    assert "OK" in a.command("MOTOR START")
    stopped(a.snapshot())
    a.advance(240, velocity=0)
    speed = a.snapshot()
    assert speed["state"] == runtime.SPEED and speed["voltage"] == 2400 and speed["period"] != 100, speed
    assert "OK" in a.command("MOTOR RPM 40")
    assert a.snapshot()["rpm_limit"] == 40 and a.snapshot()["active"]
    assert "OK" in a.command("MOTOR SPEED 10")
    assert a.snapshot()["active"] and a.snapshot()["state"] == runtime.SPEED
    assert "OK" in a.command("MODE CAN")
    assert "OK" in a.command("MODE UART")
    switched = a.snapshot()
    stopped(switched)
    assert switched["period"] == 100 and not switched["active"], switched
    assert "OK" in a.command("MOTOR START")
    legacy = a.snapshot()
    assert legacy["voltage"] == 2400 and legacy["period"] == 100, legacy
    assert "OK" in a.command("MOTOR RPM 30")
    stopped(a.snapshot())
    assert not a.snapshot()["active"]
    return {"speed": speed, "legacy": legacy}


def move_position_zero_and_parse():
    a = App()
    assert "OK" in a.command("MOTOR ZERO")
    assert "OK" in a.command("MOTOR MOVE +90")
    s = a.snapshot()
    assert s["active"] and s["position_mode"] and s["target"] == 900, s
    assert "ERR ZERO" in a.command("MOTOR ZERO")
    for bad in ["MOTOR MOVE 360001", "MOTOR POS -360001", "MOTOR MOVE +", "MOTOR POS 1.5",
                "MOTOR POS 12 X", "MOTOR RPM 351", "MOTOR RPM 2", "MOTOR RPM -30",
                "MOTOR RPM 65536", "MOTOR RPM 30 X", "MOTOR ZERO X"]:
        before = a.snapshot()
        assert "ERR" in a.command(bad), bad
        assert a.snapshot() == before, (bad, before, a.snapshot())
    assert "OK" in a.command("MOTOR POS -720")
    assert a.snapshot()["target"] == -7200
    assert "OK" in a.command("MOTOR STOP")
    stopped(a.snapshot())
    assert "OK" in a.command("MOTOR POS 360000")
    assert a.snapshot()["target"] == 3600000
    assert "OK" in a.command("MOTOR STOP")
    # New output coordinate +90 degrees, measured from raw encoder movement.
    a.now += 1000
    a.raw = -16384
    a.sample()
    assert a.snapshot()["position"] == 900
    assert "OK" in a.command("MOTOR MOVE -45")
    assert a.snapshot()["target"] == 450
    assert "OK" in a.command("MOTOR STOP")
    a.advance(400, velocity=0)
    assert "OK" in a.command("MOTOR ZERO")
    assert a.snapshot()["position"] == a.snapshot()["target"] == 0
    return {"position_and_zero": a.snapshot()}


def direction_reversal_and_position_cancel():
    a = App()
    a.start()
    assert "OK" in a.command("MOTOR DIR REV")
    waiting = a.snapshot()
    stopped(waiting)
    assert waiting["state"] == runtime.WAIT and waiting["active"], waiting
    a.advance(200, velocity=0)
    stopped(a.snapshot())
    a.advance(40, velocity=0)
    reverse = a.snapshot()
    assert reverse["voltage"] == -2400 and reverse["drive"] == -1, reverse
    assert "OK" in a.command("MOTOR MOVE 90")
    stopped(a.snapshot())
    assert "OK" in a.command("MOTOR DIR FWD")
    cancelled = a.snapshot()
    stopped(cancelled)
    assert not cancelled["active"] and not cancelled["position_mode"], cancelled
    a.advance(400, velocity=0)
    stopped(a.snapshot())
    # Position direction follows its target, independently of the speed DIR.
    assert "OK" in a.command("MOTOR MOVE -90")
    a.advance(240, velocity=0)
    negative_move = a.snapshot()
    assert negative_move["drive"] == -1 and negative_move["voltage"] == -2400
    text = a.command("STATUS")
    assert "direction_request=FWD" in text and "drive=REV" in text, text
    assert "OK" in a.command("MOTOR STOP")
    assert "drive=OFF" in a.command("STATUS")
    return {"reverse_wait": waiting, "reverse_running": reverse, "cancelled": cancelled,
            "negative_move_status": text}


def source_switch_stop_and_can():
    a = App()
    a.start()
    assert "OK" in a.command("MOTOR MOVE 360")
    assert "OK" in a.command("MODE CAN")
    stopped(a.snapshot())
    assert not a.snapshot()["active"] and a.snapshot()["period"] == 100
    a.advance(400, velocity=0)
    stopped(a.snapshot())
    a.call("Test_Can", -1800, a.now - 40)
    a.call("Test_Run", a.now)
    can = a.snapshot()
    assert can["voltage"] == -1800 and can["command_ms"] == a.now - 40, can
    assert "OK" in a.command("MODE UART")
    stopped(a.snapshot())
    assert "OK" in a.command("MOTOR RPM 20")
    assert "OK" in a.command("MOTOR START")
    assert "OK" in a.command("MOTOR STOP")
    a.advance(400, velocity=0)
    stopped(a.snapshot())
    assert not a.snapshot()["active"]
    return {"can_command_timestamp": can, "stop_cancels_wait": a.snapshot()}


def faults_require_explicit_recovery():
    a = App()
    a.start()
    a.now += 20
    a.sample(fault=5)
    failed = a.snapshot()
    stopped(failed)
    assert failed["fault"] == 3 and failed["driver_fault"] == 5
    assert "ERR" in a.command("MOTOR START")
    assert "OK" in a.command("MOTOR STOP")
    assert "OK fault cleared" in a.command("FAULT CLEAR")
    cleared = a.snapshot()
    stopped(cleared)
    assert cleared["fault"] == 0 and not cleared["active"], cleared
    a.advance(300, velocity=0)
    stopped(a.snapshot())
    assert "OK" in a.command("MOTOR START")
    a.advance(240, velocity=0)
    assert a.snapshot()["voltage"] > 0
    a.now += 100
    a.call("Test_Run", a.now)
    stale = a.snapshot()
    stopped(stale)
    assert stale["fault"] == 1 and not stale["active"]
    a.sample()
    assert "ERR" in a.command("MOTOR START")
    a.advance(300, velocity=0)
    stopped(a.snapshot())
    return {"driver_fault": failed, "clear_does_not_start": cleared, "encoder_stale": stale}


def nonblocking_status_preserves_feedback_and_stale_stop():
    a = App()
    a.start()
    a.block_txe_after_byte = True
    start_ms = a.now
    a.command("STATUS", drain=False)
    assert a.now == start_ms, "STATUS advanced time through a blocking UART call"
    assert len(a.tx) <= 1
    # Slow the wire to one byte/ms. Encoder samples continue every 20 ms while
    # a complete STATUS response drains over hundreds of foreground calls.
    for i in range(1000):
        a.now += 1
        a.uc.mem_write(a.symbols["mock_usart3"], struct.pack("<I", 1 << 7))
        if i % 20 == 0:
            a.sample()
        else:
            a.call("Test_Run", a.now)
        s = a.snapshot()
        assert s["fault"] == 0 and s["active"] and s["voltage"] > 0, s
        assert s["command_ms"] == a.now, s
    text = a.tx.decode("ascii")
    assert "motion=ENCODER" in text and "BKIN live" in text and "BKIN last" in text, text
    healthy = a.snapshot()
    # A wedged console transmitter must not delay loss-of-feedback shutdown.
    a.uc.mem_write(a.symbols["mock_usart3"], struct.pack("<I", 0))
    a.command("STATUS", drain=False)
    for _ in range(4):
        a.command("HELP", drain=False)
    # Queue saturation can drop diagnostic text but must not wait for TXE.
    assert a.snapshot()["active"] and a.snapshot()["fault"] == 0
    a.now += 100
    a.call("Test_Run", a.now)
    stale = a.snapshot()
    stopped(stale)
    assert stale["fault"] == 1 and not stale["active"], stale
    # With TXE always ready, the work budget remains bounded per loop.
    a.block_txe_after_byte = False
    a.uc.mem_write(a.symbols["mock_usart3"], struct.pack("<I", 1 << 7))
    before = len(a.tx)
    a.call("Test_Run", a.now)
    assert 0 < len(a.tx) - before <= 16
    b = App()
    help_text = b.command("HELP")
    assert "MOTOR RPM" in help_text and "MOTOR MOVE" in help_text
    b.start()
    b.uc.mem_write(b.symbols["mock_usart3"], struct.pack("<I", 0))
    for _ in range(4):
        b.command("HELP", drain=False)
    b.command("MOTOR STOP", drain=False)
    stopped(b.snapshot())
    assert not b.snapshot()["active"], "Full diagnostic queue blocked STOP"
    return {"during_status": healthy, "lost_encoder_with_blocked_tx": stale}


def duty_updates_preserve_motion_mode():
    a = App()
    a.start()
    speed_outputs = {}
    for percent in [20, 50, 100]:
        assert "OK" in a.command(f"MOTOR SPEED {percent}")
        s = a.snapshot()
        assert s["active"] and s["state"] == runtime.SPEED and not s["position_mode"], s
        assert s["voltage"] == 24000 * percent // 100, s
        speed_outputs[str(percent)] = s["voltage"]
    for bad in ["MOTOR SPEED 101", "MOTOR SPEED 7", "MOTOR SPEED -10", "MOTOR SPEED 100 X"]:
        before = a.snapshot()
        assert "ERR" in a.command(bad)
        assert a.snapshot() == before
    assert "OK" in a.command("MOTOR MOVE 720")
    a.advance(240, velocity=0)
    position_outputs = {}
    for percent in [20, 50, 100]:
        assert "OK" in a.command(f"MOTOR SPEED {percent}")
        s = a.snapshot()
        assert s["active"] and s["position_mode"] and s["state"] == runtime.POSITION, s
        assert s["target"] == 7200 and s["voltage"] == 24000 * percent // 100, s
        position_outputs[str(percent)] = s["voltage"]
    assert "OK" in a.command("MOTOR SPEED 0")
    stopped(a.snapshot())
    assert not a.snapshot()["active"]
    assert "ERR" in a.command("MOTOR START")
    assert "ERR" in a.command("MOTOR MOVE 90")
    assert "OK" in a.command("MOTOR RPM 30")
    assert "ERR" in a.command("MOTOR START")
    assert "ERR" in a.command("MOTOR POS 90")
    stopped(a.snapshot())
    assert "OK" in a.command("MOTOR SPEED 50")
    stopped(a.snapshot())
    assert "OK" in a.command("MOTOR START")
    a.advance(240, velocity=0)
    assert a.snapshot()["voltage"] == 12000
    return {"speed_voltages": speed_outputs, "position_voltages": position_outputs,
            "explicit_restart": a.snapshot()}


def original_dc_servo_and_no_hall_spam():
    a = App()
    assert "OK" in a.command("DC SPEED 50")
    assert "OK" in a.command("DC START")
    assert struct.unpack("<i", a.uc.mem_read(a.symbols["dc_duty"], 4))[0] == 500
    assert "OK" in a.command("DC DIR REV")
    assert struct.unpack("<i", a.uc.mem_read(a.symbols["dc_duty"], 4))[0] == 0
    assert "OK" in a.command("DC START")
    assert struct.unpack("<i", a.uc.mem_read(a.symbols["dc_duty"], 4))[0] == -500
    assert "OK" in a.command("SERVO ANGLE -60")
    assert struct.unpack("<i", a.uc.mem_read(a.symbols["servo_pulse"], 4))[0] == 900
    text = a.command("STATUS")
    assert "HALL=" not in text and "hall=" in text
    return {"preserved": ["DC duty/direction/stop", "SERVO", "STATUS-only Hall"]}


def main():
    commands = build()
    cases = [rpm_start_and_legacy, move_position_zero_and_parse,
             direction_reversal_and_position_cancel, source_switch_stop_and_can,
             faults_require_explicit_recovery, nonblocking_status_preserves_feedback_and_stale_stop,
             duty_updates_preserve_motion_mode,
             original_dc_servo_and_no_hall_spam]
    results = []
    for case in cases:
        try:
            result = {"name": case.__name__, "passed": True, "evidence": case()}
        except Exception:
            result = {"name": case.__name__, "passed": False, "error": traceback.format_exc()}
        results.append(result)
        print(f"{'PASS' if result['passed'] else 'FAIL'} {case.__name__}", flush=True)
        if not result["passed"]: print(result["error"], flush=True)
    report = {"passed": sum(r["passed"] for r in results), "total": len(results),
              "sources": {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in [PROJECT / "usar" / "motor_app.c", PROJECT / "usar" / "motor_motion.c"]},
              "build_commands": commands, "results": results,
              "limits": ["BLDC and peripheral boundaries are mocked; physical power, waveform and mechanics are not verified."]}
    (HERE / "results.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"{report['passed']}/{report['total']} cases passed", flush=True)
    return 0 if report["passed"] == report["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

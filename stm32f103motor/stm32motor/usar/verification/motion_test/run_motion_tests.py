"""Compile production motor_motion.c with ARMCC; exercise it in Unicorn.

The plant below is deliberately simple and independent of the controller.
It proves software behavior only, not real torque, gearbox coast or accuracy.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import time
import traceback

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]
MODULE = PROJECT / "usar" / "motor_motion.c"
HEADER = MODULE.with_suffix(".h")
DEPS = Path(os.environ.get("STM32_TEST_PYTHON_DEPS", str(Path(os.environ["TEMP"]) / "stm32motor-can-test-python")))
sys.path.insert(0, str(DEPS))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

ARM_BIN = Path(os.environ.get("ARMCC_BIN", r"D:\Keil_v5\ARM\ARMCC\bin"))
ELF_PATH = HERE / "motion_mock.axf"
FIELDS = ["state", "fault", "active", "position_mode", "feedback_ready", "reference_valid",
          "direction", "drive", "rpm_limit", "interval", "speed", "ramp", "position", "target"]
IDLE, WAIT, SPEED, POSITION, SETTLING, DONE, FAULT = range(7)


def build():
    common = [str(ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M3", "--thumb", "--c99", "-O1", "-g", "-I", str(MODULE.parent)]
    objects = [HERE / "motor_motion.o", HERE / "harness.o"]
    commands = [common + ["-c", str(MODULE), "-o", str(objects[0])],
                common + ["-c", str(HERE / "harness.c"), "-o", str(objects[1])],
                [str(ARM_BIN / "armlink.exe"), "--cpu", "Cortex-M3", "--entry", "test_entry",
                 "--ro-base", "0x10000", "--rw-base", "0x20000000", "--no_remove", "--datacompressor=off",
                 *(str(p) for p in objects), "--output", str(ELF_PATH)]]
    logs = []
    for command in commands:
        result = subprocess.run(command, capture_output=True, text=True, errors="replace", cwd=HERE)
        logs.append(subprocess.list2cmdline(command) + "\n" + result.stdout + result.stderr)
        (HERE / "build.log").write_text("\n".join(logs), encoding="utf-8")
        if result.returncode:
            raise RuntimeError(logs[-1])
    return commands


class Motion:
    def __init__(self, now=0, raw=0):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
        self.uc.mem_map(0x10000, 0x100000)
        self.uc.mem_map(0x20000000, 0x20000)
        self.uc.mem_map(0x8000, 0x1000)
        self.uc.mem_write(0x8000, b"\x00\xbf")
        with ELF_PATH.open("rb") as stream:
            elf = ELFFile(stream)
            self.symbols = {s.name: int(s["st_value"]) for s in elf.get_section_by_name(".symtab").iter_symbols() if s.name}
            for section in elf.iter_sections():
                if not int(section["sh_flags"]) & 2:
                    continue
                if section["sh_type"] == "SHT_PROGBITS":
                    self.uc.mem_write(int(section["sh_addr"]), section.data())
                elif section["sh_type"] == "SHT_NOBITS":
                    self.uc.mem_write(int(section["sh_addr"]), bytes(int(section["sh_size"])))
        self.now = now & 0xFFFFFFFF
        self.raw = float(raw)
        self.velocity = 0.0  # output rpm * 10, positive is physical FWD
        self.call("Test_Reset")
        self.sample()

    def call(self, name, *args):
        for register, value in zip([UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3], args):
            self.uc.reg_write(register, int(value) & 0xFFFFFFFF)
        self.uc.reg_write(UC_ARM_REG_SP, 0x2001F000)
        self.uc.reg_write(UC_ARM_REG_LR, 0x8001)
        self.uc.emu_start(self.symbols[name] | 1, 0x8000, count=2_000_000)
        assert self.uc.reg_read(UC_ARM_REG_PC) == 0x8000, name
        return self.uc.reg_read(UC_ARM_REG_R0)

    def snapshot(self):
        self.call("Test_Snapshot")
        return dict(zip(FIELDS, struct.unpack("<14i", self.uc.mem_read(self.symbols["test_snapshot"], 56))))

    def sample(self, flags=3, ready=1, fault=0, sample_ms=None):
        self.call("Test_Encoder", round(self.raw), self.now if sample_ms is None else sample_ms, flags)
        self.call("Test_Update", self.now, ready, fault)
        return self.snapshot()

    def tick(self, velocity=None, dt=20, gain=1.0, acceleration=2500.0):
        before = self.snapshot()
        if velocity is None:
            demand = before["drive"] * (30000.0 / before["interval"]) * gain if before["drive"] else 0.0
            change = max(-acceleration * dt / 1000, min(acceleration * dt / 1000, demand - self.velocity))
            self.velocity += change
        else:
            self.velocity = velocity
        # FWD was observed as negative encoder direction on the user's motor.
        self.raw -= self.velocity * 65536 * dt / 600000
        self.now = (self.now + dt) & 0xFFFFFFFF
        return self.sample()

    def advance(self, milliseconds, **kwargs):
        trace = []
        for _ in range(milliseconds // 20):
            trace.append(self.tick(**kwargs))
        return trace

    def start(self, rpm=30, direction=1):
        assert self.call("MotorMotion_SetRpm", rpm) == 1
        self.call("MotorMotion_SetDirection", direction)
        assert self.call("MotorMotion_Start") == 1
        return self.snapshot()


def off(s):
    assert s["drive"] == s["interval"] == s["ramp"] == 0, s


def boot_and_validation():
    m = Motion()
    s = m.snapshot()
    off(s)
    assert s["state"] == IDLE and s["rpm_limit"] == 30 and s["position"] == 0, s
    m.call("Test_Reset")
    assert m.call("MotorMotion_Start") == 0
    assert m.call("MotorMotion_Move", 90, 1) == 0
    assert m.call("MotorMotion_Zero") == 0
    m.sample()
    for rpm in (1, 2, 351, 65535):
        assert m.call("MotorMotion_SetRpm", rpm) == 0
    for degrees in (-360001, 360001, -2147483648, 2147483647):
        assert m.call("MotorMotion_Move", degrees, 0) == 0
    m.sample(ready=0)
    assert m.call("MotorMotion_Start") == 0
    m.sample(ready=1, fault=5)
    assert m.call("MotorMotion_Start") == 0
    return {"boot": s, "rejected_invalid_commands": True}


def encoder_polarity_multiturn_and_wrap():
    records = []
    # Both signed turn-count crossings and the complete raw count rollover.
    for raw, velocity in ((0, 300), (0, -300), (0x80000008, 300), (0x7FFFFFF8, -300), (8, 300), (0xFFFFFFF8, -300)):
        m = Motion(raw=raw)
        s = m.advance(2000, velocity=velocity)[-1]
        assert abs(s["speed"] - velocity) <= 4, s
        assert abs(s["position"] - velocity * 12) <= 1, s
        assert s["reference_valid"] == s["feedback_ready"] == 1, s
        records.append({"raw_start": raw, "velocity": velocity, "result": s})
    return records


def speed_ramp_and_feedback():
    m = Motion()
    start = m.start()
    off(start)
    assert start["state"] == WAIT
    trace = m.advance(6000)
    for a, b in zip(trace, trace[1:]):
        assert b["fault"] == 0, b
        if a["drive"] and b["drive"]:
            assert 0 <= b["ramp"] - a["ramp"] <= 2, (a, b)
    nominal = trace[-1]
    assert 294 <= nominal["speed"] <= 306 and 98 <= nominal["interval"] <= 102, nominal
    assert m.call("MotorMotion_SetRpm", 15) == 1
    trace = m.advance(5000)
    reduced = trace[-1]
    assert reduced["fault"] == 0 and 144 <= reduced["speed"] <= 156, reduced
    assert 190 <= reduced["interval"] <= 210, reduced
    n = Motion()
    n.start()
    compensated = n.advance(10000, gain=0.9)[-1]
    assert compensated["fault"] == 0 and 292 <= compensated["speed"] <= 306, compensated
    assert compensated["interval"] < nominal["interval"], (nominal, compensated)
    return {"nominal": nominal, "reduced": reduced, "ninety_percent_plant_gain": compensated}


def reversal_waits_for_measured_stop():
    m = Motion()
    m.start()
    m.advance(5000)
    m.call("MotorMotion_SetDirection", -1)
    first = m.snapshot()
    off(first)
    assert first["state"] == WAIT
    moving = m.advance(1000, velocity=150)
    assert all(s["drive"] == 0 and s["state"] == WAIT for s in moving)
    trace = m.advance(6000)
    reverse_index = next(i for i, s in enumerate(trace) if s["drive"] == -1)
    assert reverse_index * 20 >= 200, reverse_index
    final = trace[-1]
    assert final["fault"] == 0 and -306 <= final["speed"] <= -294, final
    return {"requested_reverse": first, "reverse_delay_ms": reverse_index * 20, "steady_reverse": final}


def stale_feedback_and_latched_recovery():
    records = []
    for now in (0, 0xFFFFFF00):
        m = Motion(now=now)
        m.start()
        m.advance(400)
        last_sample = m.now
        m.now = (last_sample + 99) & 0xFFFFFFFF
        before = m.sample(sample_ms=last_sample)
        assert before["fault"] == 0 and before["active"] == 1, before
        m.now = (last_sample + 100) & 0xFFFFFFFF
        after = m.sample(sample_ms=last_sample)
        off(after)
        assert after["fault"] == 1 and after["state"] == FAULT and after["active"] == 0, after
        assert m.call("MotorMotion_ClearFault") == 0
        m.tick(velocity=0)
        recovered_sample = m.snapshot()
        off(recovered_sample)
        assert recovered_sample["fault"] == 1 and m.call("MotorMotion_Start") == 0
        assert m.call("MotorMotion_ClearFault") == 1
        off(m.snapshot())
        assert m.call("MotorMotion_Start") == 1
        records.append({"start_time": now, "before_timeout": before, "timeout": after})
    m = Motion()
    m.start()
    missing = m.sample(flags=1)
    off(missing)
    assert missing["fault"] == 1, missing
    # A delayed foreground that receives a new frame must still detect the gap.
    m = Motion()
    m.start()
    m.advance(400)
    m.now += 100
    gap = m.sample()
    off(gap)
    assert gap["feedback_ready"] == 1 and gap["fault"] == 1 and gap["active"] == 0, gap
    records.append({"fresh_frame_after_missed_deadline": gap})
    return records


def driver_tracking_jump_and_stop_timeout():
    evidence = {}
    for fault in (0, 5):
        m = Motion()
        m.start()
        m.advance(400)
        s = m.sample(ready=0 if fault == 0 else 1, fault=fault)
        off(s)
        assert s["fault"] == 3, s
        evidence[f"driver_{fault}"] = s
    m = Motion()
    m.start()
    stalled = m.advance(4000, velocity=0)[-1]
    off(stalled)
    assert stalled["fault"] == 4, stalled
    evidence["stalled"] = stalled
    m = Motion()
    m.start()
    m.advance(400)
    m.raw += 65536
    jump = m.tick(velocity=0)
    off(jump)
    assert jump["fault"] == 2 and jump["reference_valid"] == 0, jump
    m.tick(velocity=0)
    assert m.call("MotorMotion_ClearFault") == 1
    assert m.call("MotorMotion_Move", 90, 1) == 0
    m.advance(1000, velocity=0)
    assert m.call("MotorMotion_Zero") == 1
    evidence["encoder_jump"] = jump
    m = Motion()
    m.start()
    never_stops = m.advance(5200, velocity=100)[-1]
    off(never_stops)
    assert never_stops["fault"] == 6, never_stops
    evidence["never_stops"] = never_stops
    return evidence


def run_until_done(m, limit_ms=30000, **plant):
    trace = []
    for _ in range(limit_ms // 20):
        s = m.tick(**plant)
        trace.append(s)
        assert s["fault"] == 0, s
        if s["state"] == DONE:
            off(s)
            assert abs(s["position"] - s["target"]) <= 20, s
            assert abs(s["speed"]) <= 10 and s["active"] == 0, s
            return trace
    raise AssertionError({"timed_out": trace[-1]})


def relative_absolute_position_and_retarget():
    m = Motion()
    assert m.call("MotorMotion_Zero") == 1
    assert m.call("MotorMotion_Move", 90, 1) == 1
    first = run_until_done(m)
    assert first[-1]["target"] == 900
    assert any(s["state"] == SETTLING for s in first)
    assert m.call("MotorMotion_Move", -720, 1) == 1
    target = m.snapshot()["target"]
    assert target == first[-1]["position"] - 7200
    second = run_until_done(m)
    assert m.call("MotorMotion_Move", 0, 0) == 1
    third = run_until_done(m)
    assert third[-1]["target"] == 0
    assert m.call("MotorMotion_Zero") == 1
    assert m.snapshot()["position"] == m.snapshot()["target"] == 0
    assert m.call("MotorMotion_Move", 360, 1) == 1
    m.advance(800)
    assert m.call("MotorMotion_Move", -45, 0) == 1
    replacement = m.snapshot()
    off(replacement)
    assert replacement["target"] == -450 and replacement["state"] == WAIT
    last = run_until_done(m)
    return {"relative_90": first[-1], "relative_minus720": second[-1], "absolute_zero": third[-1], "retarget": last[-1]}


def position_requires_settled_feedback_not_crossing():
    m = Motion()
    assert m.call("MotorMotion_Move", 10, 0) == 1
    m.advance(240, velocity=0)
    # Move rapidly through the target band; it must coast, then correct overshoot.
    observations = []
    for _ in range(10):
        s = m.tick(velocity=150)
        observations.append(s)
        assert s["state"] != DONE, s
    assert any(s["state"] == SETTLING for s in observations), observations
    assert observations[-1]["position"] > 120, observations[-1]
    trace = run_until_done(m, acceleration=500)
    assert any(s["drive"] == -1 for s in trace), trace[-1]
    return {"crossing": observations, "after_overshoot_correction": trace[-1]}


def cancellation_zero_and_range():
    m = Motion()
    m.start()
    m.advance(3000)
    assert m.call("MotorMotion_Zero") == 0
    m.call("MotorMotion_Stop")
    stopped = m.snapshot()
    off(stopped)
    assert stopped["state"] == IDLE and stopped["active"] == 0
    assert m.call("MotorMotion_Zero") == 0
    after = m.advance(1000)[-1]
    off(after)
    assert m.call("MotorMotion_Zero") == 1
    assert m.call("MotorMotion_Move", 90, 1) == 1
    m.advance(500)
    m.call("MotorMotion_SetDirection", -1)
    canceled = m.snapshot()
    off(canceled)
    assert canceled["active"] == canceled["position_mode"] == 0 and canceled["direction"] == -1
    assert m.call("MotorMotion_SetRpm", 0) == 1
    assert m.call("MotorMotion_Start") == 0
    return {"stop": stopped, "direction_cancels_position": canceled}


def speed_extremes_and_coordinate_bounds():
    evidence = {}
    for rpm in (3, 350):
        m = Motion()
        m.start(rpm=rpm)
        trace = m.advance(42000 if rpm == 350 else 5000)
        assert all(s["fault"] == 0 for s in trace), next(s for s in trace if s["fault"])
        assert all(8 <= s["interval"] <= 2000 for s in trace if s["drive"]), trace[-1]
        # At high speed 50 us interval quantization produces alternating periods.
        average_speed = sum(s["speed"] for s in trace[-100:]) / 100
        assert abs(average_speed - rpm * 10) < max(5, rpm / 2), (rpm, average_speed, trace[-1])
        evidence[str(rpm)] = {"last": trace[-1], "last_two_second_average_x10": average_speed}
    m = Motion()
    m.raw = -1000 * 65536
    m.now = 60000
    at_limit = m.sample()
    assert at_limit["position"] == 3600000 and at_limit["reference_valid"] == 1, at_limit
    assert m.call("MotorMotion_Move", 1, 1) == 0
    assert m.call("MotorMotion_Move", -360000, 0) == 1
    assert m.snapshot()["target"] == -3600000
    m.call("MotorMotion_Stop")
    m.raw -= 65536
    m.now += 1000
    outside = m.sample()
    assert outside["reference_valid"] == 1 and outside["position"] == 3603600, outside
    assert m.call("MotorMotion_Move", 1, 1) == 0
    # Continuous-speed odometry is wider than the allowed position target.
    n = Motion()
    n.raw = -30000 * 65536
    n.now = 1800000
    halfway = n.sample()
    assert halfway["position"] == 108000000 and halfway["reference_valid"] == 1, halfway
    n.raw = -60000 * 65536
    n.now += 1800000
    overflow = n.sample()
    assert overflow["position"] == 0 and overflow["reference_valid"] == 0, overflow
    assert n.call("MotorMotion_Move", 0, 0) == 0
    evidence["coordinate_limit"] = at_limit
    evidence["beyond_position_target_range"] = outside
    evidence["odometry_overflow_requires_zero"] = overflow
    return evidence


def main():
    commands = build()
    cases = [boot_and_validation, encoder_polarity_multiturn_and_wrap, speed_ramp_and_feedback,
             reversal_waits_for_measured_stop, stale_feedback_and_latched_recovery,
             driver_tracking_jump_and_stop_timeout, relative_absolute_position_and_retarget,
             position_requires_settled_feedback_not_crossing, cancellation_zero_and_range,
             speed_extremes_and_coordinate_bounds]
    results = []
    for case in cases:
        start = time.monotonic()
        try:
            result = {"name": case.__name__, "passed": True, "evidence": case()}
        except Exception:
            result = {"name": case.__name__, "passed": False, "error": traceback.format_exc()}
        result["seconds"] = round(time.monotonic() - start, 3)
        results.append(result)
        print(f"{'PASS' if result['passed'] else 'FAIL'} {case.__name__} ({result['seconds']:.3f}s)", flush=True)
        if not result["passed"]:
            print(result["error"], flush=True)
    report = {"module_sha256": hashlib.sha256(MODULE.read_bytes()).hexdigest(),
              "header_sha256": hashlib.sha256(HEADER.read_bytes()).hexdigest(),
              "build_commands": commands, "passed": sum(r["passed"] for r in results),
              "total": len(results), "results": results,
              "limits": ["Unicorn executes the actual ARMCC controller; encoder/driver boundary is injected.",
                         "Synthetic first-order/acceleration-limited plant does not prove real motor synchronization, torque, attainable rpm or position accuracy.",
                         "UART parsing, HAL scheduling and power-stage outputs are covered separately, not by this standalone motion suite."]}
    (HERE / "results.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"{report['passed']}/{report['total']} passed; {HERE / 'results.json'}", flush=True)
    return 0 if report["passed"] == report["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

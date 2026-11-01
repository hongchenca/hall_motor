"""Execute the real default open-loop test firmware in ARMCC/Unicorn.

Only the peripheral/HAL boundary is reused from the Hall regression suite.
Expected phase outputs and timing assertions are independent of the module.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import time
import traceback

HERE = Path(__file__).resolve().parent
HALL_TEST = HERE.parent / "bldc_test"
sys.path.insert(0, str(HALL_TEST))
import run_bldc_tests as boundary

PROJECT = HERE.parents[2]
MODULE = PROJECT / "usar" / "bldc_motor.c"
CONFIG = PROJECT / "usar" / "motor_config.h"
ELF_PATH = HERE / "open_loop_mock.axf"
FIELDS = ["open_loop", "step", "interval_ticks"]
MOE, AOE, BKE, BIE, BIF = boundary.MOE, boundary.AOE, boundary.BKE, boundary.BIE, boundary.BIF
MISSING = boundary.MISSING
COMMUTATION_TICKS = 2000

# Physical outputs for the test's six steps, not values read from the module.
# Entries are (CCER, CCMR1, CCMR2 including preserved CH4), in U,V,W order.
OUTPUTS = {
    1: (0x0041, 0x5068, 0xA500),  # U PWM, V low side
    2: (0x0401, 0x0068, 0xA550),  # U PWM, W low side
    3: (0x0410, 0x6800, 0xA550),  # V PWM, W low side
    4: (0x0014, 0x6850, 0xA500),  # V PWM, U low side
    5: (0x0104, 0x0050, 0xA568),  # W PWM, U low side
    6: (0x0140, 0x5000, 0xA568),  # W PWM, V low side
}


def build():
    # Deliberately do not override MOTOR_OPEN_LOOP_TEST: verify the board's
    # checked-in default, and assert mode=1 when the compiled firmware boots.
    common = [str(boundary.ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M3", "--thumb",
              "--c99", "-O1", "-g", "-I", str(HALL_TEST / "mock"),
              "-I", str(PROJECT / "usar")]
    objects = [HERE / "bldc_motor.o", HERE / "harness.o"]
    commands = [
        common + ["-c", str(MODULE), "-o", str(objects[0])],
        common + ["-c", str(HALL_TEST / "harness.c"), "-o", str(objects[1])],
        [str(boundary.ARM_BIN / "armlink.exe"), "--cpu", "Cortex-M3", "--entry", "test_entry",
         "--ro-base", "0x10000", "--rw-base", "0x20000000", "--no_remove", "--datacompressor=off",
         str(objects[0]), str(objects[1]), "--output", str(ELF_PATH)],
    ]
    log = []
    for command in commands:
        result = subprocess.run(command, capture_output=True, text=True, errors="replace", cwd=HERE)
        log.append(subprocess.list2cmdline(command) + "\n" + result.stdout + result.stderr)
        (HERE / "build.log").write_text("\n".join(log), encoding="utf-8")
        if result.returncode:
            raise RuntimeError(f"Build failed ({result.returncode}): {log[-1]}")
    return commands


class Motor(boundary.Motor):
    def __init__(self, bkin_high=1):
        super().__init__(bkin_high=bkin_high, elf_path=ELF_PATH)
        assert self.snapshot()["open_loop"] == 1, "Default firmware is not the open-loop test mode"

    def snapshot(self):
        result = super().snapshot()
        self.call("Test_OpenLoopSnapshot")
        values = struct.unpack("<3I", self.uc.mem_read(self.symbols["test_open_loop_snapshot"], 12))
        result.update(zip(FIELDS, values))
        return result

    def start(self, voltage=5000, hall=0, sample=2500):
        self.advance(64, sample=sample, hall=hall)
        self.command(voltage)
        self.one(sample=sample, hall=hall)
        s = self.snapshot()
        assert s["state"] == 2 and s["fault"] == 0, s
        phase(s, 1)
        assert s["interval_ticks"] == COMMUTATION_TICKS, s
        return s

    def run(self, voltage=5000, hall=0):
        return self.start(voltage=voltage, hall=hall)


def off(s, state=0):
    assert s["state"] == state and s["ccer"] == 0 and not s["bdtr"] & MOE, s
    assert s["duty"] == s["ccr1"] == s["ccr2"] == s["ccr3"] == 0, s
    assert s["step"] == s["interval_ticks"] == 0, s
    assert s["bdtr"] & BKE and not s["bdtr"] & AOE, s


def phase(s, step):
    assert s["step"] == step, s
    assert (s["ccer"], s["ccmr1"], s["ccmr2"]) == OUTPUTS[step], s
    assert s["ccer"] & 0x1000 == 0, "CH4 output became enabled"
    assert s["bdtr"] & MOE and s["bdtr"] & BKE and not s["bdtr"] & AOE, s
    assert 256 <= s["duty"] <= 320, s
    assert s["duty"] == s["ccr1"] == s["ccr2"] == s["ccr3"], s


def startup_boundaries():
    m = Motor()
    boot = m.snapshot()
    off(boot)
    assert boot["fault"] == 0, boot
    running = m.start(hall=7)
    assert running["hall"] == 7, "Diagnostic Hall must not select the open-loop phase"
    m.advance(COMMUTATION_TICKS - 1, hall=0, refresh=5000)
    before_step = m.snapshot()
    phase(before_step, 1)
    assert before_step["interval_ticks"] == COMMUTATION_TICKS, before_step
    m.one(hall=0)
    first_step = m.snapshot()
    phase(first_step, 2)
    assert first_step["interval_ticks"] == COMMUTATION_TICKS, first_step
    return {"boot": boot, "immediate_step_1": running,
            "one_tick_before_100ms": before_step, "first_step_at_100ms": first_step}


def sequence_and_fixed_period(direction):
    voltage = direction * 5000
    m = Motor()
    m.run(voltage=voltage, hall=0)
    step = 1
    trace = []
    for index in range(12):
        m.advance(COMMUTATION_TICKS - 1, hall=(index % 8), refresh=voltage)
        before = m.snapshot()
        phase(before, step)
        assert before["interval_ticks"] == COMMUTATION_TICKS and before["fault"] == 0, before
        m.one(hall=((index + 1) % 8))
        step = ((step - 1 + direction) % 6) + 1
        after = m.snapshot()
        phase(after, step)
        assert after["interval_ticks"] == COMMUTATION_TICKS and after["fault"] == 0, after
        trace.append({k: after[k] for k in ["step", "interval_ticks", "ccer", "ccmr1", "ccmr2", "hall"]})
    return {"direction": direction, "steps": trace}


def hall_independence():
    fixed, changing = Motor(), Motor()
    fixed.run(hall=0)
    changing.run(hall=7)
    compared = []
    keys = ["state", "fault", "duty", "step", "interval_ticks", "ccer", "ccmr1", "ccmr2", "bdtr"]
    for i in range(16):
        count = 713 + i * 17
        fixed.advance(count, hall=0, refresh=5000)
        changing.advance(count, hall=(i % 8), refresh=5000)
        a, b = fixed.snapshot(), changing.snapshot()
        assert {k: a[k] for k in keys} == {k: b[k] for k in keys}, (a, b)
        assert a["hall"] == 0 and b["hall"] == (i % 8), (a, b)
        assert a["fault"] == b["fault"] == 0
        compared.append({"step": a["step"], "changing_hall": b["hall"]})
    fixed.advance(120000, hall=0, refresh=5000)
    changing.advance(120000, hall=7, refresh=5000)
    a, b = fixed.snapshot(), changing.snapshot()
    assert {k: a[k] for k in keys} == {k: b[k] for k in keys}, (a, b)
    assert a["state"] == 2 and a["fault"] == 0, a
    phase(a, a["step"])
    assert a["hall"] == 0 and b["hall"] == 7, (a, b)
    return {"matched_traces": compared, "running_over_5_seconds": a}


def input_duty_limit():
    result = {}
    for voltage in [5000, 32767, -32768]:
        m = Motor()
        s = m.run(voltage=voltage, hall=7)
        assert s["duty"] == 320, s
        m.advance(COMMUTATION_TICKS, hall=0, refresh=voltage)
        after = m.snapshot()
        phase(after, 2 if voltage > 0 else 6)
        result[str(voltage)] = after
    m = Motor()
    m.run(voltage=1900)
    s = m.snapshot()
    assert s["duty"] == 1900 * 3200 // s["vbus_mv"] and s["duty"] < 320, s
    result["below_cap_above_minimum"] = s
    m.command(1)
    m.one(hall=7)
    off(m.snapshot())
    result["below_minimum_stopped"] = m.snapshot()
    return result


def command_timeout(wrap=False):
    m = Motor()
    m.run()
    sent = 0xFFFFFF00 if wrap else m.snapshot()["now"]
    m.command(5000, sent)
    m.call("Test_SetTime", (sent + 499) & 0xFFFFFFFF)
    m.one(hall=0)
    before = m.snapshot()
    assert before["state"] == 2 and before["timeout"] == 0, before
    m.call("Test_SetTime", (sent + 500) & 0xFFFFFFFF)
    m.one(hall=7)
    after = m.snapshot()
    off(after)
    assert after["timeout"] == 1 and after["fault"] == 0, after
    return {"at_499ms": before, "at_500ms": after}


def stop_restart():
    m = Motor()
    m.run()
    m.call("Bldc_EmergencyStop")
    stopped = m.snapshot()
    off(stopped)
    m.advance(1000, hall=7)
    off(m.snapshot())
    m.command(5000)
    m.one(hall=0)
    restarted = m.snapshot()
    assert restarted["state"] == 2, restarted
    phase(restarted, 1)
    m.command(0)
    m.one(hall=7)
    command_stop = m.snapshot()
    off(command_stop)
    assert command_stop["fault"] == 0, command_stop
    return {"emergency_stop": stopped, "restart_at_step_1": restarted,
            "zero_command_stop": command_stop}


def reversal_coast():
    m = Motor()
    m.run()
    before = m.snapshot()
    m.command(-5000)
    m.one(hall=0)
    coast = m.snapshot()
    off(coast, state=3)
    assert coast["fault"] == before["fault"] == 0, coast
    m.advance(1999, hall=7, refresh=-5000)
    off(m.snapshot(), state=3)
    m.one(hall=0)
    restarted = m.snapshot()
    assert restarted["state"] == 2 and restarted["interval_ticks"] == COMMUTATION_TICKS, restarted
    phase(restarted, 1)
    m.advance(COMMUTATION_TICKS - 1, hall=0, refresh=-5000)
    phase(m.snapshot(), 1)
    m.one(hall=7)
    reversed_state = m.snapshot()
    phase(reversed_state, 6)
    return {"coast": coast, "restart_at_step_1": restarted,
            "first_reverse_step_after_100ms": reversed_state}


def break_lifecycle():
    m = Motor()
    m.run(hall=0)
    m.call("Test_SetBreak", 1)
    m.call("Test_DispatchBreakIrq")
    broken = m.snapshot()
    off(broken)
    assert broken["fault"] == 5 and not broken["dier"] & BIE, broken
    diagnostic = m.diagnostic()
    assert diagnostic["source"] == 3 and diagnostic["sr"] & BIF and diagnostic["hall"] == 0, diagnostic
    m.call("Test_SetBreak", 0)
    m.advance(20, hall=7)
    off(m.snapshot())
    assert m.snapshot()["fault"] == 5
    assert m.call("Bldc_ClearFault") == 1
    m.command(0)
    m.call("Test_SetBkinPin", 0)
    assert m.call("Bldc_ClearFault") == 1
    m.call("Test_SetBkinPin", 1)
    for sample in [1805, 3386]:
        m.advance(128, sample=sample, hall=0)
        assert m.call("Bldc_ClearFault") == 1, "Unsafe supply allowed fault clear"
    m.advance(128, hall=7)
    m.call("Mock_SetPrimask", 1)
    assert m.call("Bldc_ClearFault") == 0, "Hall 7 blocked test-mode clear"
    cleared = m.snapshot()
    off(cleared)
    assert cleared["fault"] == 0 and cleared["dier"] & BIE and cleared["primask"] == 1, cleared
    assert cleared["break_flag"] == 0
    m.call("Mock_SetPrimask", 0)
    m.command(5000)
    m.one(hall=0)
    assert m.snapshot()["state"] == 2
    m.call("Test_SetBreak", 1)
    m.call("Test_DispatchBreakIrq")
    off(m.snapshot())
    assert m.snapshot()["fault"] == 5
    return {"latched": broken, "diagnostic_before_hal_clear": diagnostic, "explicit_clear": cleared, "protection_retriggered": m.snapshot()}


def software_break_during_run():
    m = Motor()
    m.start()
    m.call("Test_SetBkinPin", 0)
    m.one(hall=7)
    s = m.snapshot()
    off(s)
    assert s["fault"] == 5, s
    assert m.diagnostic()["source"] == 2 and m.diagnostic()["state"] == 2
    return {"stopped": s, "diagnostic": m.diagnostic()}


def startup_break_conditions():
    boot = Motor(bkin_high=0)
    initial = boot.snapshot()
    off(initial)
    assert initial["fault"] == 5 and not initial["dier"] & BIE
    m = Motor()
    m.prime()
    m.call("Test_SetBreak", 1)
    m.call("Test_SetBreak", 0)
    m.command(5000)
    m.one(hall=0)
    pending = m.snapshot()
    off(pending)
    assert pending["fault"] == 5 and m.diagnostic()["source"] == 4
    short = Motor()
    short.prime()
    event = short.inject_break_during_moe_delay()
    short.command(5000)
    short.one(hall=7)
    s = short.snapshot()
    off(s)
    assert s["fault"] == 5 and event["injected"] and event["released"], (s, event)
    assert short.diagnostic()["source"] == 5
    return {"low_at_boot": initial, "pending_bif": pending, "short_break_in_moe_wait": s}


def adc_missing_and_dma_error():
    m = Motor()
    m.run(hall=0)
    m.advance(40, sample=MISSING, hall=7)
    before = m.snapshot()
    assert before["fault"] == 0 and before["ready"] == 1
    m.one(sample=MISSING, hall=0)
    lost = m.snapshot()
    off(lost)
    assert lost["fault"] == 6 and lost["ready"] == 0
    m.command(0)
    assert m.call("Bldc_ClearFault") == 1
    m.advance(63, hall=0)
    assert m.snapshot()["ready"] == 0
    m.one(hall=7)
    assert m.snapshot()["ready"] == 1 and m.snapshot()["fault"] == 6
    assert m.call("Bldc_ClearFault") == 0
    off(m.snapshot())
    dma = Motor()
    dma.run(hall=7)
    dma.call("Test_DmaError")
    damaged = dma.snapshot()
    off(damaged)
    assert damaged["fault"] == 6 and damaged["ready"] == 0
    dma.command(0)
    dma.advance(128, hall=0)
    assert dma.call("Bldc_ClearFault") == 1, "DMA error cleared without reinitialization"
    return {"missing_40": before, "missing_41": lost, "fresh_pair_recovery": m.snapshot(), "dma_error": damaged}


def voltage_protection():
    result = {}
    for sample, fault in [(1805, 2), (3386, 3)]:
        m = Motor()
        m.run()
        m.advance(320, sample=sample, hall=7, refresh=5000)
        s = m.snapshot()
        off(s)
        assert s["fault"] == fault, s
        m.command(0)
        assert m.call("Bldc_ClearFault") == 1
        result[f"running_{sample}"] = s
        boot = Motor()
        boot.advance(64, sample=sample, hall=0)
        boot.command(5000)
        boot.one(sample=sample, hall=7)
        off(boot.snapshot())
        boot.advance(240, sample=sample, hall=0)
        final = boot.snapshot()
        off(final)
        assert final["fault"] == fault, final
        result[f"startup_{sample}"] = final
    return result


def initialization_failures():
    result = {}
    for variable in ["mock_hal_timer_result", "mock_hal_timer_it_result", "mock_hal_calibration_result", "mock_hal_adc_result"]:
        m = Motor()
        m.set_word(variable, 1)
        assert m.call("Bldc_Init") == 1
        s = m.snapshot()
        off(s)
        if "adc" in variable or "calibration" in variable:
            assert s["fault"] == 6
        result[variable] = s
    return result


def main():
    commands = build()
    cases = [
        ("start_enters_step_1_and_first_commutation_at_100ms", startup_boundaries),
        ("forward_six_steps_fixed_100ms_period", lambda: sequence_and_fixed_period(1)),
        ("reverse_six_steps_fixed_100ms_period", lambda: sequence_and_fixed_period(-1)),
        ("hall_zero_seven_changed_and_stalled_do_not_control_output", hall_independence),
        ("signed_voltage_entrypoint_ten_percent_cap", input_duty_limit),
        ("command_timeout_499_500_ms", command_timeout),
        ("command_timeout_uint32_wrap", lambda: command_timeout(True)),
        ("stop_restart_and_zero_command_stop", stop_restart),
        ("running_reversal_coast_restart_and_reverse_sequence", reversal_coast),
        ("break_latch_clear_ignores_hall_but_keeps_other_guards", break_lifecycle),
        ("software_pb12_guard_during_run", software_break_during_run),
        ("boot_pending_bif_and_short_break_guards", startup_break_conditions),
        ("adc_missing_40_41_and_dma_error", adc_missing_and_dma_error),
        ("under_overvoltage_block_start_and_stop_running", voltage_protection),
        ("hal_initialization_failures_outputs_off", initialization_failures),
    ]
    results = []
    for name, function in cases:
        started = time.monotonic()
        try:
            result = {"name": name, "passed": True, "evidence": function()}
        except Exception:
            result = {"name": name, "passed": False, "error": traceback.format_exc()}
        result["seconds"] = round(time.monotonic() - started, 3)
        results.append(result)
        print(f"{'PASS' if result['passed'] else 'FAIL'} {name} ({result['seconds']:.3f}s)", flush=True)
        if not result["passed"]:
            print(result["error"], flush=True)
    report = {
        "module": str(MODULE),
        "module_sha256": hashlib.sha256(MODULE.read_bytes()).hexdigest(),
        "config_sha256": hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
        "mode": "Default firmware (MOTOR_OPEN_LOOP_TEST=1 required at runtime)",
        "method": "ARMCC compiles actual bldc_motor.c; Unicorn executes Thumb; only HAL/register storage is mocked",
        "build_commands": commands,
        "passed": sum(r["passed"] for r in results),
        "total": len(results),
        "results": results,
        "limits": [
            "No physical motor motion, current, thermal limit, synchronization or mechanical speed is verified.",
            "A PWM duty cap is not a current limit; this firmware has no runtime cutoff or stall detector.",
            "GPIO values and ADC completions are injected; PWM waveforms, real ADC noise and asynchronous IRQ timing are not simulated.",
            "Signed voltage tests exercise the shared BLDC command entrypoint, not UART parsing or CAN reception.",
            "Break injection verifies software guards and mock register behavior, not power-stage fault timing.",
        ],
    }
    (HERE / "results.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"{report['passed']}/{report['total']} cases passed; {HERE / 'results.json'}", flush=True)
    return 0 if report["passed"] == report["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

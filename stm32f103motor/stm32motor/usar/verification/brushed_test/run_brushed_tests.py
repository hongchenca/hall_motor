"""Compile the real actuator driver with ARMCC and execute it in Unicorn."""
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
MODULE = PROJECT / "usar" / "actuators.c"
HEADER = PROJECT / "usar" / "actuators.h"
DEPS = Path(os.environ.get("STM32_TEST_PYTHON_DEPS", str(Path(os.environ["TEMP"]) / "stm32motor-can-test-python")))
sys.path.insert(0, str(DEPS))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_PC, UC_ARM_REG_LR

ARM_BIN = Path(os.environ.get("ARMCC_BIN", r"D:\Keil_v5\ARM\ARMCC\bin"))
ELF_PATH = HERE / "brushed_mock.axf"
PIN_IN1, PIN_IN2 = 1 << 4, 1 << 5


def build():
    common = [str(ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M3", "--thumb", "--c99", "-O1", "-g", "-I", str(HERE / "mock"), "-I", str(PROJECT / "usar")]
    commands = [
        common + ["-c", str(MODULE), "-o", str(HERE / "actuators.o")],
        common + ["-c", str(HERE / "harness.c"), "-o", str(HERE / "harness.o")],
        [str(ARM_BIN / "armlink.exe"), "--cpu", "Cortex-M3", "--entry", "test_entry", "--ro-base", "0x10000", "--rw-base", "0x20000000", "--no_remove", "--datacompressor=off", str(HERE / "actuators.o"), str(HERE / "harness.o"), "--output", str(ELF_PATH)],
    ]
    log = []
    for command in commands:
        result = subprocess.run(command, capture_output=True, text=True, errors="replace", cwd=HERE)
        log.append(subprocess.list2cmdline(command) + "\n" + result.stdout + result.stderr)
        if result.returncode:
            (HERE / "build.log").write_text("\n".join(log), encoding="utf-8")
            raise RuntimeError(f"Build failed ({result.returncode}): {log[-1]}")
    (HERE / "build.log").write_text("\n".join(log), encoding="utf-8")
    return commands


class Actuator:
    def __init__(self):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
        self.uc.mem_map(0x10000, 0x100000)
        self.uc.mem_map(0x20000000, 0x20000)
        self.uc.mem_map(0x8000, 0x1000)
        self.uc.mem_write(0x8000, b"\x00\xbf")
        with ELF_PATH.open("rb") as stream:
            elf = ELFFile(stream)
            self.symbols = {item.name: int(item["st_value"]) for item in elf.get_section_by_name(".symtab").iter_symbols() if item.name}
            for section in elf.iter_sections():
                if not (int(section["sh_flags"]) & 2):
                    continue
                address = int(section["sh_addr"])
                if section["sh_type"] == "SHT_PROGBITS":
                    self.uc.mem_write(address, section.data())
                elif section["sh_type"] == "SHT_NOBITS":
                    self.uc.mem_write(address, bytes(int(section["sh_size"])))
        assert self.call("Test_Reset") == 0

    def call(self, name, *args):
        for register, value in zip([UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3], args):
            self.uc.reg_write(register, value & 0xFFFFFFFF)
        self.uc.reg_write(UC_ARM_REG_SP, 0x2001F000)
        self.uc.reg_write(UC_ARM_REG_LR, 0x8001)
        try:
            self.uc.emu_start(self.symbols[name] | 1, 0x8000, count=10_000_000)
        except UcError as error:
            raise RuntimeError(f"{name}: {error}; PC={self.uc.reg_read(UC_ARM_REG_PC):08x}") from error
        assert self.uc.reg_read(UC_ARM_REG_PC) == 0x8000, f"{name}: instruction limit reached"
        return self.uc.reg_read(UC_ARM_REG_R0)

    def set_word(self, name, value):
        self.uc.mem_write(self.symbols[name], struct.pack("<I", value))

    def snapshot(self):
        self.call("Test_Snapshot")
        values = struct.unpack("<40I", self.uc.mem_read(self.symbols["test_snapshot"], 40 * 4))
        return {"pins": values[0], "pwm": values[1], "servo": values[2], "arr": values[3], "egr": values[4], "starts": values[5], "stops": values[6], "trace_count": values[7], "trace": [values[8 + index * 4:12 + index * 4] for index in range(8)]}

    def init(self):
        assert self.call("Test_Init") == 0


def stopped(s):
    assert s["pwm"] == 0 and (s["pins"] & (PIN_IN1 | PIN_IN2)) == 0, s


def init_starts_off():
    a = Actuator()
    a.init()
    s = a.snapshot()
    assert s["pins"] == 0 and s["pwm"] == 0 and s["servo"] == 1500, s
    assert s["starts"] == 2 and s["arr"] == 3199, s
    return s


def signed_direction_and_pwm_mapping():
    a = Actuator()
    a.init()
    points = []
    for duty, pins, pwm in [(1, PIN_IN1, 3), (300, PIN_IN1, 960), (1000, PIN_IN1, 3200), (1500, PIN_IN1, 3200), (-250, PIN_IN2, 800), (-1000, PIN_IN2, 3200), (-2000, PIN_IN2, 3200)]:
        a.call("Test_Duty", duty)
        s = a.snapshot()
        assert s["pins"] == pins and s["pwm"] == pwm, {"duty": duty, **s}
        assert s["pins"] in (PIN_IN1, PIN_IN2), s
        points.append({"duty_per_mille": duty, "pins": s["pins"], "pwm": s["pwm"]})
    return points


def reversal_clears_pwm_before_direction():
    a = Actuator()
    a.init()
    a.call("Test_Duty", 700)
    a.call("Test_ClearTrace")
    a.call("Test_Duty", -500)
    s = a.snapshot()
    assert s["pins"] == PIN_IN2 and s["pwm"] == 1600, s
    assert s["trace_count"] == 2, s
    # Each direction write observes a committed zero PWM compare.
    assert all(entry[2] == 0 and entry[3] == 1 for entry in s["trace"][:2]), s
    assert s["trace"][0][0:2] == (PIN_IN1 | PIN_IN2, 0), s
    assert s["trace"][1][0:2] == (PIN_IN2, 1), s
    a.call("Test_ClearTrace")
    a.call("Test_Duty", -250)
    same_direction = a.snapshot()
    assert same_direction["trace_count"] == 0 and same_direction["pins"] == PIN_IN2, same_direction
    return {"reversed_to": s, "same_direction_update": same_direction}


def zero_and_explicit_stop():
    a = Actuator()
    a.init()
    a.call("Test_Duty", -640)
    a.call("Test_Duty", 0)
    zero = a.snapshot()
    stopped(zero)
    a.call("Test_Duty", 450)
    a.call("Test_Stop")
    stop = a.snapshot()
    stopped(stop)
    return {"zero_duty": zero, "explicit_stop": stop}


def failed_init_stays_inert():
    a = Actuator()
    a.set_word("mock_pwm_fail_call", 1)
    assert a.call("Test_Init") == 1
    a.call("Test_Duty", 500)
    s = a.snapshot()
    assert s["pins"] == 0 and s["pwm"] == 0, s
    return s


def main():
    commands = build()
    cases = [
        ("initialization_keeps_brushed_output_off", init_starts_off),
        ("signed_direction_and_duty_to_pwm_mapping", signed_direction_and_pwm_mapping),
        ("reversal_commits_zero_before_direction_write", reversal_clears_pwm_before_direction),
        ("zero_duty_and_stop_clear_pwm_and_both_pins", zero_and_explicit_stop),
        ("pwm_start_failure_leaves_driver_inert", failed_init_stays_inert),
    ]
    results = []
    for name, function in cases:
        started = time.monotonic()
        try:
            results.append({"name": name, "passed": True, "evidence": function(), "seconds": round(time.monotonic() - started, 3)})
        except Exception:
            results.append({"name": name, "passed": False, "error": traceback.format_exc(), "seconds": round(time.monotonic() - started, 3)})
        result = results[-1]
        print(f"{'PASS' if result['passed'] else 'FAIL'} {name} ({result['seconds']:.3f}s)", flush=True)
        if not result["passed"]:
            print(result["error"], flush=True)
    report = {
        "module": str(MODULE),
        "module_sha256": hashlib.sha256(MODULE.read_bytes()).hexdigest(),
        "header_sha256": hashlib.sha256(HEADER.read_bytes()).hexdigest(),
        "method": "ARMCC compiles actual actuators.c; Unicorn executes Cortex-M3 Thumb; HAL and GPIO/TIM storage mocked.",
        "build_commands": commands,
        "passed": sum(case["passed"] for case in results),
        "total": len(results),
        "results": results,
        "limits": ["Register and GPIO writes are modeled; this does not measure real PWM pin waveforms, motor response, H-bridge timing, or electrical transients.", "The test assumes TIM3_CH4 PWM1/high polarity and ARR=3199 as configured by this project."],
    }
    (HERE / "results.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"{report['passed']}/{report['total']} cases passed; {HERE / 'results.json'}", flush=True)
    return 0 if report["passed"] == report["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

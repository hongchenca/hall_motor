"""Compile the real bldc_motor.c with ARMCC, then execute it in Unicorn.

Only HAL calls and register storage are mocked. No BLDC algorithm is copied.
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
MODULE = PROJECT / "usar" / "bldc_motor.c"
CONFIG = PROJECT / "usar" / "motor_config.h"
DEPS = Path(os.environ.get("STM32_TEST_PYTHON_DEPS", str(Path(os.environ["TEMP"]) / "stm32motor-can-test-python")))
sys.path.insert(0, str(DEPS))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

ARM_BIN = Path(os.environ.get("ARMCC_BIN", r"D:\Keil_v5\ARM\ARMCC\bin"))
ELF_PATH = HERE / "bldc_mock.axf"
BASELINE = HERE / "fixtures" / "pre_moe_sync"
BASELINE_MODULE = BASELINE / "bldc_motor.c"
BASELINE_ELF_PATH = HERE / "bldc_moe_baseline.axf"
MISSING = 0xFFFF
NO_REFRESH = 0x10000
MOE, AOE, BKE, BIE, BIF = 1 << 15, 1 << 14, 1 << 12, 1 << 7, 1 << 7
FIELDS = ["state", "fault", "vbus_adc", "vbus_mv", "duty", "hall", "ready", "timeout", "ccer", "ccmr1", "ccmr2", "bdtr", "ccr1", "ccr2", "ccr3", "dier", "now", "primask", "pa2_adc", "dma_length", "dma_ccr", "bkin_high", "break_flag"]
DIAGNOSTIC_FIELDS = ["source", "sr", "bdtr", "dier", "pb12", "hall", "state"]
PHASES = {
    1: (0x0140, 0x5000, 0x0068),
    2: (0x0014, 0x6850, 0x0000),
    3: (0x0104, 0x0050, 0x0068),
    4: (0x0401, 0x0068, 0x0050),
    5: (0x0041, 0x5068, 0x0000),
    6: (0x0410, 0x6800, 0x0050),
}


def build():
    # Select the retained Hall firmware even when the board build selects the
    # separate open-loop test mode. No protection feature is overridden.
    common = [str(ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M3", "--thumb", "--c99", "-O1", "-g", "-DMOTOR_OPEN_LOOP_TEST=0", "-I", str(HERE / "mock"), "-I", str(PROJECT / "usar")]
    module_object = HERE / "bldc_motor.o"
    harness_object = HERE / "harness.o"
    commands = [
        common + ["-c", str(MODULE), "-o", str(module_object)],
        common + ["-c", str(HERE / "harness.c"), "-o", str(harness_object)],
        [str(ARM_BIN / "armlink.exe"), "--cpu", "Cortex-M3", "--entry", "test_entry", "--ro-base", "0x10000", "--rw-base", "0x20000000", "--no_remove", "--datacompressor=off", str(module_object), str(harness_object), "--output", str(ELF_PATH)],
    ]
    # Unmodified pre-fix source/harness in the checked-in fixture, separately
    # built for a synthetic timing comparison; not a production build source.
    baseline_common = [str(ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M3", "--thumb", "--c99", "-O1", "-g", "-I", str(HERE / "mock"), "-I", str(BASELINE), "-I", str(PROJECT / "usar")]
    commands += [
        baseline_common + ["-c", str(BASELINE_MODULE), "-o", str(HERE / "bldc_motor_moe_baseline.o")],
        baseline_common + ["-c", str(BASELINE / "harness.c"), "-o", str(HERE / "harness_moe_baseline.o")],
        [str(ARM_BIN / "armlink.exe"), "--cpu", "Cortex-M3", "--entry", "test_entry", "--ro-base", "0x10000", "--rw-base", "0x20000000", "--no_remove", "--datacompressor=off", str(HERE / "bldc_motor_moe_baseline.o"), str(HERE / "harness_moe_baseline.o"), "--output", str(BASELINE_ELF_PATH)],
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


class Motor:
    def __init__(self, bkin_high=1, elf_path=ELF_PATH):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
        self.uc.mem_map(0x10000, 0x100000)
        self.uc.mem_map(0x20000000, 0x20000)
        self.uc.mem_map(0x8000, 0x1000)
        self.uc.mem_write(0x8000, b"\x00\xbf")
        with elf_path.open("rb") as stream:
            elf = ELFFile(stream)
            self.symbols = {s.name: int(s["st_value"]) for s in elf.get_section_by_name(".symtab").iter_symbols() if s.name}
            # ARMCC stores RW initialization bytes beside RO bytes in one
            # PT_LOAD segment. Load each allocated section at its execution
            # address, reproducing the CRT scatter-copy and zero-init steps.
            for section in elf.iter_sections():
                if not (int(section["sh_flags"]) & 2):
                    continue
                address = int(section["sh_addr"])
                if section["sh_type"] == "SHT_PROGBITS":
                    self.uc.mem_write(address, section.data())
                elif section["sh_type"] == "SHT_NOBITS":
                    self.uc.mem_write(address, bytes(int(section["sh_size"])))
        self.timer_reads = 0
        timer_cnt = self.symbols["mock_tim2"] + 9 * 4

        def tick_timer(uc, access, address, size, value, user_data):
            # A 1 MHz free-running timer advances between volatile CNT reads.
            current = struct.unpack("<I", uc.mem_read(timer_cnt, 4))[0]
            uc.mem_write(timer_cnt, struct.pack("<I", (current + 1) & 0xFFFF))
            self.timer_reads += 1

        self.uc.hook_add(UC_HOOK_MEM_READ, tick_timer, begin=timer_cnt, end=timer_cnt + 3)
        assert self.call("Test_ResetWithBkin", bkin_high) == 0

    def call(self, function, *args):
        for register, value in zip([UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3], args):
            self.uc.reg_write(register, value & 0xFFFFFFFF)
        self.uc.reg_write(UC_ARM_REG_SP, 0x2001F000)
        self.uc.reg_write(UC_ARM_REG_LR, 0x8001)
        try:
            self.uc.emu_start(self.symbols[function] | 1, 0x8000, count=150_000_000)
        except UcError as error:
            raise RuntimeError(f"{function}: {error}; PC={self.uc.reg_read(UC_ARM_REG_PC):08x}") from error
        assert self.uc.reg_read(UC_ARM_REG_PC) == 0x8000, f"{function}: instruction limit reached"
        return self.uc.reg_read(UC_ARM_REG_R0)

    def set_word(self, symbol, value):
        self.uc.mem_write(self.symbols[symbol], struct.pack("<I", value))

    def snapshot(self):
        self.call("Test_Snapshot")
        data = self.uc.mem_read(self.symbols["test_snapshot"], len(FIELDS) * 4)
        return dict(zip(FIELDS, struct.unpack("<" + "I" * len(FIELDS), data)))

    def diagnostic(self):
        self.call("Test_BreakDiagnostic")
        data = self.uc.mem_read(self.symbols["test_break_diagnostic"], len(DIAGNOSTIC_FIELDS) * 4)
        return dict(zip(DIAGNOSTIC_FIELDS, struct.unpack("<" + "I" * len(DIAGNOSTIC_FIELDS), data)))

    def read_register(self, symbol, offset=0):
        return struct.unpack("<I", self.uc.mem_read(self.symbols[symbol] + offset, 4))[0]

    def write_register(self, symbol, offset, value):
        self.uc.mem_write(self.symbols[symbol] + offset, struct.pack("<I", value))

    def model_moe_readback_delay(self, required_timer_reads=2):
        """Artificial model, NOT a cycle-accurate STM32 or a hardware proof.

        A 0->1 MOE write becomes readable only after the specified number of
        TIM2 CNT reads. This makes the required intervening delay observable.
        """
        bdtr = self.symbols["mock_tim1"] + 17 * 4
        model = {"required_timer_reads": required_timer_reads, "pending": False,
                 "moe_write_timer_reads": 0, "first_read_after_write": None}

        def write_bdtr(uc, access, address, size, value, user_data):
            previous = struct.unpack("<I", uc.mem_read(bdtr, 4))[0]
            if value & MOE and not previous & MOE:
                model["pending"] = True
                model["moe_write_timer_reads"] = self.timer_reads
            elif not value & MOE:
                model["pending"] = False

        def read_bdtr(uc, access, address, size, value, user_data):
            if not model["pending"]:
                return
            elapsed = self.timer_reads - model["moe_write_timer_reads"]
            if model["first_read_after_write"] is None:
                model["first_read_after_write"] = elapsed
            current = struct.unpack("<I", uc.mem_read(bdtr, 4))[0]
            if elapsed >= required_timer_reads:
                uc.mem_write(bdtr, struct.pack("<I", current | MOE))
                model["pending"] = False
            else:
                uc.mem_write(bdtr, struct.pack("<I", current & ~MOE))

        self.uc.hook_add(UC_HOOK_MEM_WRITE, write_bdtr, begin=bdtr, end=bdtr + 3)
        self.uc.hook_add(UC_HOOK_MEM_READ, read_bdtr, begin=bdtr, end=bdtr + 3)
        return model

    def inject_break_during_moe_delay(self):
        """Inject a short hardware Break during the delay, with no IRQ callback."""
        bdtr = self.symbols["mock_tim1"] + 17 * 4
        timer = self.symbols["mock_tim2"] + 9 * 4
        event = {"armed": False, "injected": False, "released": False}

        def arm(uc, access, address, size, value, user_data):
            if value & MOE and not event["injected"]:
                event["armed"] = True

        def pulse(uc, access, address, size, value, user_data):
            if event["armed"] and not event["injected"]:
                self.write_register("mock_gpiob", 0, self.read_register("mock_gpiob") & ~(1 << 12))
                self.write_register("mock_tim1", 17 * 4, self.read_register("mock_tim1", 17 * 4) & ~MOE)
                self.write_register("mock_tim1", 4 * 4, self.read_register("mock_tim1", 4 * 4) | BIF)
                event["injected"] = True
            elif event["injected"] and not event["released"]:
                self.write_register("mock_gpiob", 0, self.read_register("mock_gpiob") | (1 << 12))
                event["released"] = True

        self.uc.hook_add(UC_HOOK_MEM_WRITE, arm, begin=bdtr, end=bdtr + 3)
        self.uc.hook_add(UC_HOOK_MEM_READ, pulse, begin=timer, end=timer + 3)
        return event

    def advance(self, count, sample=2500, hall=1, refresh=NO_REFRESH):
        self.call("Test_Advance", count, sample, hall, refresh)

    def one(self, sample=2500, hall=1):
        self.call("Test_One", sample, hall)

    def command(self, voltage, timestamp=None):
        if timestamp is None:
            timestamp = self.snapshot()["now"]
        self.call("Bldc_SetVoltageMv", voltage, timestamp)

    def prime(self, sample=2500):
        self.advance(64, sample)

    def run(self, voltage=5000):
        self.prime()
        self.command(voltage)
        self.one()
        self.advance(2001)
        s = self.snapshot()
        assert s["state"] == 2 and s["fault"] == 0 and s["bdtr"] & MOE, s
        return s


def off(s):
    assert s["state"] == 0 and s["ccer"] == 0 and (s["bdtr"] & MOE) == 0, s
    assert s["duty"] == 0 and s["ccr1"] == s["ccr2"] == s["ccr3"] == 0, s


def phase(s, index):
    ccer, ccmr1, ccmr2 = PHASES[index]
    assert (s["ccer"], s["ccmr1"], s["ccmr2"]) == (ccer, ccmr1, 0xA500 | ccmr2), s
    assert s["ccer"] & 0x1000 == 0, "TIM1 CH4 output became enabled"
    assert s["bdtr"] & MOE and not s["bdtr"] & AOE, s


def adc_zero():
    m = Motor()
    m.command(5000)
    m.advance(64, 0)
    s = m.snapshot()
    assert s["ready"] == 1 and s["vbus_adc"] == s["vbus_mv"] == 0, s
    off(s)
    m.advance(240, 0)
    assert m.snapshot()["fault"] == 2
    return {"zero_bus": s, "confirmed_undervoltage": m.snapshot()}


def adc_average():
    m = Motor()
    m.advance(32, 2000)
    m.advance(31, 3000)
    before = m.snapshot()
    assert before["ready"] == 0 and before["vbus_mv"] == 0
    m.one(3000)
    after = m.snapshot()
    assert after["ready"] == 1 and after["vbus_adc"] == 2500
    assert after["vbus_mv"] == 2500 * 30000 // 3385
    off(after)
    return {"samples_63": before, "samples_64": after}


def dma_channel_separation():
    m = Motor()
    initial = m.snapshot()
    assert initial["dma_length"] == 2 and initial["dma_ccr"] == 0xB, initial
    samples = []
    for pa2 in [4095, 0, 1234]:
        m.set_word("mock_pa2_sample", pa2)
        m.advance(64, 2500)
        s = m.snapshot()
        assert s["vbus_adc"] == 2500 and s["pa2_adc"] == pa2 and s["fault"] == 0, s
        assert s["vbus_mv"] == 2500 * 30000 // 3385, s
        samples.append(s)
    m.set_word("mock_pa2_sample", 4095)
    m.advance(300, 1805)
    low = m.snapshot()
    assert low["fault"] == 2 and low["pa2_adc"] == 4095, low
    off(low)
    return {"pa2_variations_do_not_change_bus": samples, "pa1_still_controls_undervoltage": low}


def dma_partial_pair():
    m = Motor()
    m.call("Test_HalfAdc", 4095)
    m.advance(10, MISSING)
    partial = m.snapshot()
    assert partial["ready"] == 0 and partial["vbus_adc"] == partial["pa2_adc"] == 0, partial
    m.advance(63, 2000)
    assert m.snapshot()["ready"] == 0
    m.one(2000)
    full = m.snapshot()
    assert full["ready"] == 1 and full["vbus_adc"] == 2000 and full["pa2_adc"] == 1234, full
    return {"partial_not_consumed": partial, "64_complete_pairs": full}


def dma_transfer_error():
    m = Motor()
    m.run()
    m.command(0)
    m.call("Test_DmaError")
    failed = m.snapshot()
    off(failed)
    assert failed["fault"] == 6 and failed["ready"] == 0, failed
    assert m.call("Bldc_ClearFault") == 1
    m.advance(64)
    blocked = m.snapshot()
    assert blocked["fault"] == 6 and blocked["ready"] == 0, blocked
    assert m.call("Bldc_ClearFault") == 1
    off(blocked)
    return {"immediate_stop": failed, "requires_reinitialization": blocked}


def forward_phases():
    m = Motor()
    m.prime()
    m.command(5000)
    m.one()
    pre = m.snapshot()
    assert pre["state"] == 1 and pre["ccer"] == 0x444 and pre["duty"] == 0 and pre["bdtr"] & MOE
    m.advance(2000)
    assert m.snapshot()["state"] == 1
    m.one()
    found = []
    for hall in range(1, 7):
        m.one(hall=hall)
        s = m.snapshot()
        phase(s, hall)
        assert s["duty"] == 5000 * 3200 // s["vbus_mv"]
        assert s["ccr1"] == s["ccr2"] == s["ccr3"] == s["duty"]
        found.append({key: s[key] for key in ["hall", "ccer", "ccmr1", "ccmr2", "duty"]})
    return {"precharge": pre, "forward": found}


def reverse_phases():
    m = Motor()
    m.run(-5000)
    found = []
    for hall in range(1, 7):
        m.one(hall=hall)
        s = m.snapshot()
        phase(s, 7 - hall)
        found.append({key: s[key] for key in ["hall", "ccer", "ccmr1", "ccmr2", "duty"]})
    return {"reverse": found}


def negative_minimum():
    m = Motor()
    s = m.run(-32768)
    phase(s, 6)
    assert s["duty"] == s["ccr1"] == s["ccr2"] == s["ccr3"] == 3200, s
    return s


def reversal_coast():
    m = Motor()
    m.run()
    m.command(-5000)
    m.one()
    coast = m.snapshot()
    assert coast["state"] == 3 and coast["ccer"] == 0 and not coast["bdtr"] & MOE
    m.advance(2000)
    assert m.snapshot()["state"] == 3 and not m.snapshot()["bdtr"] & MOE
    m.one()
    assert m.snapshot()["state"] == 1
    m.advance(2000)
    assert m.snapshot()["state"] == 1
    m.one()
    final = m.snapshot()
    assert final["state"] == 2
    phase(final, 6)
    return {"coasting": coast, "running_reverse": final}


def command_timeout(wrap=False):
    m = Motor()
    m.run()
    sent = 0xFFFFFF00 if wrap else m.snapshot()["now"]
    m.command(5000, sent)
    m.call("Test_SetTime", (sent + 499) & 0xFFFFFFFF)
    m.one()
    at_499 = m.snapshot()
    assert at_499["state"] == 2 and at_499["timeout"] == 0
    m.call("Test_SetTime", (sent + 500) & 0xFFFFFFFF)
    m.one()
    at_500 = m.snapshot()
    off(at_500)
    assert at_500["timeout"] == 1 and at_500["fault"] == 0
    return {"received_ms": sent, "elapsed_499": at_499, "elapsed_500": at_500}


def break_latch_clear():
    m = Motor()
    m.run()
    m.call("Test_SetBreak", 1)
    m.call("Test_DispatchBreakIrq")
    broken = m.snapshot()
    off(broken)
    assert broken["fault"] == 5 and broken["dier"] & BIE == 0
    assert broken["bkin_high"] == 0 and broken["bdtr"] & BKE, broken
    m.call("Test_SetBreak", 0)
    m.advance(20)
    off(m.snapshot())
    assert m.call("Bldc_ClearFault") == 1, "nonzero command allowed fault clear"
    m.command(0)
    m.call("Test_SetBreak", 1)
    assert m.call("Bldc_ClearFault") == 1, "active BKIN allowed fault clear"
    m.call("Test_SetBreak", 0)
    for hall in [0, 7]:
        m.call("Test_SetHall", hall)
        assert m.call("Bldc_ClearFault") == 1, f"Hall {hall} allowed fault clear"
    m.call("Test_SetHall", 1)
    for sample in [1805, 3386]:
        m.advance(128, sample)
        assert m.call("Bldc_ClearFault") == 1, f"unsafe ADC {sample} allowed fault clear"
    m.advance(128)
    m.call("Mock_SetPrimask", 1)
    assert m.call("Bldc_ClearFault") == 0
    cleared = m.snapshot()
    off(cleared)
    assert cleared["fault"] == 0 and cleared["dier"] & BIE and cleared["primask"] == 1
    assert cleared["bdtr"] & BKE and cleared["break_flag"] == 0, cleared
    assert cleared["bdtr"] & AOE == 0
    m.call("Mock_SetPrimask", 0)
    m.one()
    off(m.snapshot())
    m.command(5000)
    m.one()
    assert m.snapshot()["state"] == 1
    m.advance(2001)
    restarted = m.snapshot()
    assert restarted["state"] == 2 and restarted["fault"] == 0, restarted
    assert restarted["bdtr"] & MOE and restarted["bdtr"] & BKE and restarted["dier"] & BIE, restarted
    m.call("Test_SetBreak", 1)
    m.call("Test_DispatchBreakIrq")
    retriggered = m.snapshot()
    off(retriggered)
    assert retriggered["fault"] == 5 and not retriggered["dier"] & BIE, retriggered
    assert retriggered["bkin_high"] == 0 and retriggered["bdtr"] & BKE, retriggered
    return {"latched": broken, "explicitly_cleared_still_off": cleared,
            "new_command_restarted": restarted, "protection_retriggered_after_clear": retriggered}


def pending_break_before_start():
    m = Motor()
    m.prime()
    m.call("Test_SetBreak", 1)
    m.call("Test_SetBreak", 0)
    m.command(5000)
    m.one()
    s = m.snapshot()
    off(s)
    assert s["fault"] == 5
    d = m.diagnostic()
    assert d["source"] == 4 and d["sr"] & BIF and d["pb12"] == 1, d
    assert d["state"] == 1, d
    return {"stopped": s, "enable_precheck_diagnostic": d}


def bkin_low_at_initialization():
    m = Motor(bkin_high=0)
    initial = m.snapshot()
    off(initial)
    assert initial["bkin_high"] == 0 and initial["fault"] == 5, initial
    assert initial["bdtr"] & BKE and not initial["dier"] & BIE, initial
    first_diagnostic = m.diagnostic()
    assert first_diagnostic["source"] == 1 and first_diagnostic["pb12"] == 0, first_diagnostic
    assert first_diagnostic["hall"] == 1 and first_diagnostic["state"] == 0, first_diagnostic
    m.prime()
    m.command(0)
    assert m.call("Bldc_ClearFault") == 1, "PB12 low at boot allowed fault clear"
    denied = m.snapshot()
    off(denied)
    assert denied["fault"] == 5 and denied["bkin_high"] == 0, denied
    assert m.diagnostic() == first_diagnostic, "failed clear or FastTick replaced initial evidence"
    m.call("Test_SetBkinPin", 1)
    m.one()
    recovered_pin = m.snapshot()
    off(recovered_pin)
    assert recovered_pin["fault"] == 5, "PB12 high automatically cleared boot fault"
    assert m.call("Bldc_ClearFault") == 0
    cleared = m.snapshot()
    off(cleared)
    assert cleared["fault"] == 0 and cleared["bkin_high"] == 1, cleared
    assert cleared["bdtr"] & BKE and cleared["dier"] & BIE, cleared
    assert cleared["break_flag"] == 0, cleared
    assert all(v == 0 for v in m.diagnostic().values()), m.diagnostic()
    m.advance(20)
    off(m.snapshot())
    return {"pb12_low_at_boot": initial, "clear_refused_while_low": denied,
            "high_does_not_clear_latch": recovered_pin, "explicit_clear_still_off": cleared,
            "first_init_diagnostic_retained_until_clear": first_diagnostic}


def bkin_low_runtime_software_latch():
    m = Motor()
    running = m.run()
    assert running["bkin_high"] == 1 and running["bdtr"] & BKE and running["dier"] & BIE, running
    # Change only the GPIO: no simulated hardware action and no Break IRQ.
    # This isolates the normal FastTick low-active software guard.
    m.call("Test_SetBkinPin", 0)
    before_tick = m.snapshot()
    assert before_tick["fault"] == 0 and before_tick["break_flag"] == 0, before_tick
    assert before_tick["bdtr"] & MOE, before_tick
    m.one()
    stopped = m.snapshot()
    off(stopped)
    assert stopped["fault"] == 5 and stopped["bkin_high"] == 0, stopped
    d = m.diagnostic()
    assert d["source"] == 2 and d["pb12"] == 0 and d["state"] == 2, d
    assert d["bdtr"] & MOE and d["sr"] & BIF == 0, d
    m.call("Test_SetBkinPin", 1)
    m.advance(20)
    high_again = m.snapshot()
    off(high_again)
    assert high_again["fault"] == 5, high_again
    assert m.diagnostic() == d, "subsequent stop changed the first diagnostic"
    return {"pin_low_before_software_tick": before_tick,
            "software_guard_latched": stopped, "pin_high_does_not_restart": high_again,
            "captured_before_software_outputs_off": d}


def irq_diagnostic_lifecycle():
    m = Motor()
    m.run()
    m.call("Test_SetBreak", 1)
    m.call("Bldc_CaptureBreakIrq")
    captured = m.diagnostic()
    before_callback = m.snapshot()
    assert captured["source"] == 3 and captured["sr"] & BIF and captured["dier"] & BIE, captured
    assert captured["pb12"] == 0 and captured["hall"] == 1 and captured["state"] == 2, captured
    assert before_callback["fault"] == 0 and before_callback["state"] == 2, before_callback
    assert before_callback["ccer"] != 0, "capture unexpectedly changed the software output state"
    m.call("Test_DispatchBreakIrq")
    stopped = m.snapshot()
    off(stopped)
    assert stopped["fault"] == 5 and stopped["break_flag"] == 0, stopped
    assert m.diagnostic() == captured, "HAL clearing BIF destroyed the IRQ evidence"
    m.one()  # A later low-pin software check must not replace IRQ evidence.
    assert m.diagnostic() == captured
    m.call("Test_SetBkinPin", 1)
    m.advance(20)
    assert m.diagnostic() == captured
    assert m.snapshot()["fault"] == 5
    m.command(0)
    m.call("Test_SetHall", 0)
    assert m.call("Bldc_ClearFault") == 1
    assert m.diagnostic() == captured, "rejected clear erased the first evidence"
    m.call("Test_SetHall", 1)
    assert m.call("Bldc_ClearFault") == 0
    cleared = m.diagnostic()
    assert all(value == 0 for value in cleared.values()), cleared
    m.run()
    m.call("Test_SetBreak", 1)
    m.call("Test_DispatchBreakIrq")
    assert m.diagnostic()["source"] == 3
    assert m.call("Test_ResetWithBkin", 1) == 0
    reinitialized = m.diagnostic()
    assert all(value == 0 for value in reinitialized.values()), reinitialized
    return {"capture_before_hal_clears_bif": captured, "capture_only_does_not_latch": before_callback,
            "callback_latches_with_live_bif_cleared": stopped, "successful_clear_resets": cleared,
            "initialization_resets": reinitialized}


def invalid_irq_capture_ignored():
    m = Motor()
    m.run()
    m.call("Bldc_CaptureBreakIrq")  # BIE=1, BIF=0.
    no_flag = m.diagnostic()
    assert no_flag["source"] == 0, no_flag
    m.write_register("mock_tim1", 3 * 4, m.read_register("mock_tim1", 3 * 4) & ~BIE)
    m.call("Test_SetBreak", 1)  # BIF=1, BIE=0.
    m.call("Bldc_CaptureBreakIrq")
    disabled = m.diagnostic()
    assert disabled["source"] == 0, disabled
    assert m.snapshot()["fault"] == 0
    return {"no_bif_not_captured": no_flag, "disabled_bie_not_captured": disabled}


def break_during_moe_wait():
    m = Motor()
    m.prime()
    m.command(5000)
    event = m.inject_break_during_moe_delay()
    m.one()
    s = m.snapshot()
    off(s)
    assert event["injected"] and event["released"], event
    assert s["fault"] == 5 and s["bkin_high"] == 1 and s["break_flag"] == 1, s
    d = m.diagnostic()
    assert d["source"] == 5 and d["pb12"] == 1 and d["sr"] & BIF, d
    assert not d["bdtr"] & MOE and d["bdtr"] & BKE and d["state"] == 1, d
    return {"artificial_short_pulse": event, "stopped_even_after_pin_recovers": s,
            "enable_postcheck_diagnostic": d}


def synthetic_moe_sync_comparison():
    evidence = {}
    for label, path in [("pre_fix_unmodified_fixture", BASELINE_ELF_PATH), ("current_production", ELF_PATH)]:
        m = Motor(elf_path=path)
        m.prime()
        m.command(5000)
        model = m.model_moe_readback_delay(required_timer_reads=2)
        m.one()
        s = m.snapshot()
        assert s["bkin_high"] == 1 and s["break_flag"] == 0, s
        if path == BASELINE_ELF_PATH:
            off(s)
            assert s["fault"] == 5 and model["first_read_after_write"] == 0, (s, model)
        else:
            assert s["fault"] == 0 and s["state"] == 1 and s["bdtr"] & MOE, s
            assert model["first_read_after_write"] >= 2, model
            assert m.diagnostic()["source"] == 0
        evidence[label] = {"artificial_readback_model": dict(model), "snapshot": s}
    evidence["limit"] = "Synthetic TIM2-read-gated MOE readback model; not measured STM32 latency or proof of the board fault's cause."
    return evidence


def adc_loss():
    m = Motor()
    m.run()
    m.advance(40, MISSING)
    at_40 = m.snapshot()
    assert at_40["fault"] == 0 and at_40["ready"] == 1
    m.one(MISSING)
    lost = m.snapshot()
    off(lost)
    assert lost["fault"] == 6 and lost["ready"] == 0
    m.command(0)
    assert m.call("Bldc_ClearFault") == 1
    m.advance(63)
    assert m.snapshot()["ready"] == 0, "stale samples survived ADC timeout"
    m.one()
    recovered_adc = m.snapshot()
    off(recovered_adc)
    assert recovered_adc["ready"] == 1 and recovered_adc["fault"] == 6
    assert m.call("Bldc_ClearFault") == 0
    off(m.snapshot())
    return {"missing_40": at_40, "missing_41": lost, "fresh_samples_fault_still_latched": recovered_adc}


def invalid_hall(hall):
    m = Motor()
    m.run()
    m.one(hall=hall)
    s = m.snapshot()
    off(s)
    assert s["fault"] == 1
    return s


def stalled_hall():
    m = Motor()
    m.run()
    m.advance(65534, refresh=5000)
    before = m.snapshot()
    assert before["state"] == 2 and before["fault"] == 0 and before["timeout"] == 0, before
    m.one()
    stalled = m.snapshot()
    off(stalled)
    assert stalled["fault"] == 1 and stalled["timeout"] == 0
    return {"unchanged_hall_65534": before, "unchanged_hall_65535": stalled, "timer_counter_reads": m.timer_reads}


def overvoltage_start_blocked():
    m = Motor()
    m.prime(4000)
    m.command(5000)
    m.one(4000)
    before = m.snapshot()
    off(before)
    assert before["fault"] == 0
    m.advance(240, 4000)
    final = m.snapshot()
    off(final)
    assert final["fault"] == 3
    return {"before_confirmation_outputs_off": before, "confirmed": final}


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
        ("adc_zero_no_divide_or_output", adc_zero),
        ("64_sample_average_exact_boundary", adc_average),
        ("dma_pa1_pa2_separation_and_bus_protection", dma_channel_separation),
        ("dma_partial_pair_not_consumed", dma_partial_pair),
        ("dma_error_stops_and_requires_reinitialization", dma_transfer_error),
        ("precharge_then_six_forward_hall_phases", forward_phases),
        ("six_reverse_hall_phases", reverse_phases),
        ("signed_minus_32768_saturates_safely", negative_minimum),
        ("reversal_coast_and_reprecharge", reversal_coast),
        ("command_timeout_499_and_500_ms", command_timeout),
        ("command_timeout_across_uint32_wrap", lambda: command_timeout(True)),
        ("break_latch_clear_conditions_no_automatic_moe_restart", break_latch_clear),
        ("pending_break_flag_blocks_start", pending_break_before_start),
        ("pb12_low_at_boot_latches_and_requires_safe_explicit_clear", bkin_low_at_initialization),
        ("pb12_low_runtime_software_guard_latches_without_irq", bkin_low_runtime_software_latch),
        ("irq_first_snapshot_before_hal_clear_and_reset_lifecycle", irq_diagnostic_lifecycle),
        ("irq_snapshot_requires_both_bif_and_bie", invalid_irq_capture_ignored),
        ("short_break_during_moe_wait_still_stops", break_during_moe_wait),
        ("synthetic_moe_readback_delay_old_vs_current", synthetic_moe_sync_comparison),
        ("adc_missing_40_and_41_ticks_latched_recovery", adc_loss),
        ("hall_zero_stops_and_latches", lambda: invalid_hall(0)),
        ("hall_seven_stops_and_latches", lambda: invalid_hall(7)),
        ("unchanged_hall_stall_65535_ticks", stalled_hall),
        ("overvoltage_blocks_start_then_latches", overvoltage_start_blocked),
        ("hal_initialization_failures_leave_outputs_off", initialization_failures),
    ]
    results = []
    for name, function in cases:
        started = time.monotonic()
        try:
            evidence = function()
            result = {"name": name, "passed": True, "evidence": evidence}
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
        "mode": "Hall commutation (MOTOR_OPEN_LOOP_TEST=0)",
        "synthetic_comparison_baseline_module": str(BASELINE_MODULE),
        "synthetic_comparison_baseline_sha256": hashlib.sha256(BASELINE_MODULE.read_bytes()).hexdigest(),
        "method": "ARMCC compiles actual bldc_motor.c; Unicorn executes Thumb; HAL and register storage mocked; TIM2 CNT increments per read",
        "build_commands": commands,
        "passed": sum(item["passed"] for item in results),
        "total": len(results),
        "results": results,
        "limits": ["No real gate-driver, motor, ADC timing/noise or electrical break latency is simulated.", "Stable Hall values are injected; asynchronous IRQ preemption and unstable/glitching Hall waveforms are not exercised.", "Register storage records writes; PWM pin waveforms, timer preload transfer and external pull-down behavior are not simulated.", "ADC completion is injected per FastTick; this verifies application state transitions rather than real ADC conversion scheduling.", "The MOE readback comparison uses an explicit artificial TIM2-read-gated model, not measured STM32 synchronization latency and not proof of the board fault's cause.", "The Break-in-delay case injects register/pin changes at a timer read and the IRQ wrapper reproduces capture/clear/callback ordering; neither simulates hardware electrical timing or the complete HAL vector."],
    }
    (HERE / "results.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"{report['passed']}/{report['total']} cases passed; {HERE / 'results.json'}", flush=True)
    return 0 if report["passed"] == report["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

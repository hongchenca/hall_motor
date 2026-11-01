"""Build unmodified can_bus.c using ARMCC 5 and execute its ARM ELF in Unicorn."""
from pathlib import Path
import argparse
import hashlib
import io
import json
import os
import subprocess
import sys

HERE = Path(__file__).resolve().parent
MODULE = HERE.parent.parent / 'can_bus.c'
DEPS = Path(os.environ.get('TEMP', '')) / 'stm32motor-can-test-python'
if DEPS.is_dir():
    sys.path.insert(0, str(DEPS))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP, UC_ARM_REG_LR

def build(keil):
    build_dir = HERE / 'build'
    build_dir.mkdir(exist_ok=True)
    armcc, armlink = keil / 'bin' / 'armcc.exe', keil / 'bin' / 'armlink.exe'
    common = ['--cpu', 'Cortex-M3', '--thumb', '--c99', '-O0', '-g',
              '-I' + str(HERE / 'mock'), '-I' + str(MODULE.parent)]
    commands = [
        [str(armcc), *common, '-c', str(MODULE), '-o', str(build_dir / 'can_bus.o')],
        [str(armcc), *common, '-c', str(HERE / 'can_test.c'), '-o', str(build_dir / 'can_test.o')],
        [str(armlink), '--cpu', 'Cortex-M3', '--entry', 'Test_Run', '--ro_base', '0x10000',
         '--rw_base', '0x20000000', '--no_scanlib', '--output', str(build_dir / 'can_test.axf'),
         str(build_dir / 'can_bus.o'), str(build_dir / 'can_test.o')],
    ]
    log = []
    for command in commands:
        result = subprocess.run(command, capture_output=True, text=True)
        log.append({'argv': command, 'returncode': result.returncode,
                    'stdout': result.stdout, 'stderr': result.stderr})
        if result.returncode:
            (HERE / 'build_log.json').write_text(json.dumps(log, indent=2), encoding='utf-8')
            raise RuntimeError(result.stdout + result.stderr)
    (HERE / 'build_log.json').write_text(json.dumps(log, indent=2), encoding='utf-8')
    return build_dir / 'can_test.axf'

def run_case(blob, name, mode, motor):
    elf = ELFFile(io.BytesIO(blob))
    symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.mem_map(0, 0x100000)
    uc.mem_map(0x20000000, 0x20000)
    # ARMCC packs initialized RW load bytes after RO in one PT_LOAD. Load
    # SHF_ALLOC sections at execution addresses to model startup scatter-copy.
    for section in elf.iter_sections():
        if section['sh_flags'] & 2 and section['sh_type'] == 'SHT_PROGBITS':
            uc.mem_write(section['sh_addr'], section.data())
    sentinel = 0x8000
    uc.mem_write(sentinel, b'\x00\xbe')
    uc.reg_write(UC_ARM_REG_SP, 0x20010000)
    uc.reg_write(UC_ARM_REG_LR, sentinel | 1)
    uc.reg_write(UC_ARM_REG_R0, mode)
    uc.reg_write(UC_ARM_REG_R1, motor)
    uc.hook_add(UC_HOOK_CODE, lambda machine, address, size, data:
                machine.emu_stop() if address == sentinel else None)
    uc.emu_start(symbols['Test_Run'] | 1, sentinel + 2, count=2000000)
    read = lambda key: int.from_bytes(uc.mem_read(symbols[key], 4), 'little')
    line, checks, done = read('test_failed_line'), read('test_checks'), read('test_done')
    return {'name': name, 'mode': mode, 'motor_id': motor, 'checks': checks,
            'failed_line': line, 'completed': done == 1, 'pass': line == 0 and done == 1}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--keil', type=Path, default=Path(r'D:\Keil_v5\ARM\ARMCC'))
    args = parser.parse_args()
    elf_path = build(args.keil)
    blob = elf_path.read_bytes()
    scenarios = [('protocol_node_%d' % node, 0, node) for node in range(1, 9)]
    scenarios += [('filter_failure', 1, 1), ('start_timeout', 2, 1),
                  ('notification_failure', 3, 1), ('cleanup_notification_failure', 4, 1),
                  ('cleanup_stop_failure', 5, 1), ('invalid_instance_state', 6, 1),
                  ('tick_wrap', 7, 8), ('pending_irq_on_start', 8, 1),
                  ('pending_irq_failed_start_cleanup', 9, 1)]
    cases = [run_case(blob, *case) for case in scenarios]
    report = {'module': str(MODULE), 'module_sha256': hashlib.sha256(MODULE.read_bytes()).hexdigest(),
              'elf_sha256': hashlib.sha256(blob).hexdigest(), 'cases': cases,
              'total_checks': sum(case['checks'] for case in cases),
              'pass': all(case['pass'] for case in cases)}
    output = HERE / 'results.json'
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    for case in cases:
        print(('PASS' if case['pass'] else 'FAIL'), case['name'],
              'checks=%d failed_line=%d' % (case['checks'], case['failed_line']))
    print('Total:', report['total_checks'], 'checks;', output)
    return 0 if report['pass'] else 1

if __name__ == '__main__':
    sys.exit(main())

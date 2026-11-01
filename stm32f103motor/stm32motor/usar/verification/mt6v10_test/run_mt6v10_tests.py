"""Compile the real MT_6V10 Modbus module and execute it in Unicorn."""
from pathlib import Path
import argparse
import hashlib
import io
import json
import os
import subprocess
import sys

HERE = Path(__file__).resolve().parent
MODULE = HERE.parent.parent / 'mt6v10_modbus.c'
DEPS = Path(os.environ.get('TEMP', '')) / 'stm32motor-can-test-python'
if DEPS.is_dir():
    sys.path.insert(0, str(DEPS))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_SP, UC_ARM_REG_LR


def build(keil):
    build_dir = HERE / 'build'
    build_dir.mkdir(exist_ok=True)
    armcc, armlink = keil / 'bin' / 'armcc.exe', keil / 'bin' / 'armlink.exe'
    common = ['--cpu', 'Cortex-M3', '--thumb', '--c99', '-O0', '-g',
              '-I' + str(HERE / 'mock'), '-I' + str(MODULE.parent)]
    commands = [
        [str(armcc), *common, '-c', str(MODULE), '-o', str(build_dir / 'mt6v10_modbus.o')],
        [str(armcc), *common, '-c', str(HERE / 'harness.c'), '-o', str(build_dir / 'harness.o')],
        [str(armlink), '--cpu', 'Cortex-M3', '--entry', 'Test_Run', '--ro_base', '0x10000',
         '--rw_base', '0x20000000', '--output', str(build_dir / 'mt6v10_test.axf'),
         str(build_dir / 'mt6v10_modbus.o'), str(build_dir / 'harness.o')],
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
    return build_dir / 'mt6v10_test.axf'


def run(blob):
    elf = ELFFile(io.BytesIO(blob))
    symbols = {symbol.name: symbol['st_value']
               for symbol in elf.get_section_by_name('.symtab').iter_symbols()}
    machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    machine.mem_map(0, 0x100000)
    machine.mem_map(0x20000000, 0x20000)
    for section in elf.iter_sections():
        if section['sh_flags'] & 2 and section['sh_type'] == 'SHT_PROGBITS':
            machine.mem_write(section['sh_addr'], section.data())
    sentinel = 0x8000
    machine.mem_write(sentinel, b'\x00\xbe')
    machine.reg_write(UC_ARM_REG_SP, 0x20010000)
    machine.reg_write(UC_ARM_REG_LR, sentinel | 1)
    machine.reg_write(UC_ARM_REG_R0, 0)
    machine.hook_add(UC_HOOK_CODE, lambda uc, address, size, data:
                     uc.emu_stop() if address == sentinel else None)
    machine.emu_start(symbols['Test_Run'] | 1, sentinel + 2, count=4000000)
    read = lambda name: int.from_bytes(machine.mem_read(symbols[name], 4), 'little')
    return {'checks': read('test_checks'), 'failed_line': read('test_failed_line'),
            'completed': read('test_done') == 1}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--keil', type=Path, default=Path(r'D:\Keil_v5\ARM\ARMCC'))
    args = parser.parse_args()
    elf_path = build(args.keil)
    blob = elf_path.read_bytes()
    result = run(blob)
    result.update({'module': str(MODULE),
                   'module_sha256': hashlib.sha256(MODULE.read_bytes()).hexdigest(),
                   'elf_sha256': hashlib.sha256(blob).hexdigest(),
                   'pass': result['failed_line'] == 0 and result['completed']})
    output = HERE / 'results.json'
    output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(('PASS' if result['pass'] else 'FAIL'), 'MT_6V10 Modbus',
          'checks=%d failed_line=%d' % (result['checks'], result['failed_line']))
    print('Result:', output)
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())

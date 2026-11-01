# CAN module execution tests

`run_can_test.py` compiles the real, unchanged `../../can_bus.c` as its own
ARMCC 5 object and links it to the fake HAL in `can_test.c`. Unicorn executes
the resulting Cortex-M3 Thumb machine code. Each scenario starts in a fresh
emulator, with zeroed RAM and initialized ELF sections copied to their runtime
addresses, so private module state is reset without test-only changes.

Replay from PowerShell in the firmware project root (`stm32f103motor/stm32motor`), with Python on PATH:

```powershell
python .\usar\verification\can_test\run_can_test.py
```

The default compiler directory is `D:\Keil_v5\ARM\ARMCC`; override it with
`--keil PATH`. Dependencies are `unicorn` and `pyelftools`. The script also
searches `$env:TEMP\stm32motor-can-test-python`, where they were installed for
this workspace. On another machine install them with the selected Python:

```powershell
python -m pip install unicorn pyelftools
```

The runner writes `results.json` with the module SHA-256, ELF SHA-256, scenario
results and assertion counts. `build_log.json` records exact compiler/linker
arguments and diagnostics. Objects and the executable are under `build/`.
Any failed assertion records its line in `can_test.c` and returns exit code 1.

Coverage:

- Node IDs 1 through 8, command IDs 0x100/0x200, all four command slots,
  literal little-endian payloads including negative values and int16 limits.
- Captured 32-bit bxCAN filter words accept only the intended standard data
  ID. Callback additionally rejects wrong IDs, extended frames, remote frames
  and DLC other than 8. FIFO draining, newest command wins, foreign handles,
  NULL outputs, read failures and retry are exercised.
- Feedback ID 0x100 + motor ID, signed angle/speed byte encoding, zero tail,
  2 ms period, mailbox contention, transmit failure and tick rollover.
- IRQ mask save/restore with both initial mask values. A pending callback
  delivered synchronously while notifications are enabled proves initialized
  state is already published. Failed initialization then clears that command.
- Filter/start/notification errors, start timeout, cleanup errors, inactive
  transmission after failure, and successful retry after simulated MX_CAN_Init.
- Retention of a command for over 500 ms proves timeout is delegated to the
  application, as specified by `can_bus.h`.

These tests mock the HAL and do not emulate bxCAN registers, physical CAN
arbitration, interrupt timing, transceivers or motor hardware. The complete
Keil project build and on-board CAN/motor checks remain separate verification.

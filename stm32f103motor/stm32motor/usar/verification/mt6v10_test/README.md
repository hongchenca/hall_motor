# MT_6V10 Modbus tests

The runner compiles the real `usar/mt6v10_modbus.c` with ARMCC 5 and executes
the Cortex-M3 code in Unicorn. Only the RS485 byte transport is mocked.

Run from PowerShell in the firmware project root (`stm32f103motor/stm32motor`), with Python on PATH:

```powershell
python .\usar\verification\mt6v10_test\run_mt6v10_tests.py
```

The test checks the documented Modbus CRC example, the exact FC03 request for
registers 1 and 2, CRC validation, register byte order, signed multi-turn
conversion, CAN feedback units, speed direction, response timeout, and stale
link handling. `results.json` records the source and test ELF hashes plus the
assertion result. This does not test USART interrupts or physical RS485 wiring.

# Motion / UART application integration regression

Run from the firmware project directory:

```powershell
py usar/verification/motion_app_test/run_motion_app_tests.py
```

ARMCC compiles the production `motor_app.c` and `motor_motion.c`; Unicorn executes
the Cortex-M3 instructions. The harness feeds command bytes through the actual
UART command parser and observes calls at the BLDC, encoder, CAN and actuator
boundaries. No production parser or motion-control algorithm is copied.

The eight cases cover:

- RPM configuration, explicit START, and transition to/from legacy duty mode.
- MOVE/POS/ZERO coordinates, limits, malformed input and argument rejection.
- Reversal waiting with outputs off and cancellation of a position move by DIR.
- UART/CAN ownership changes, preserved CAN receive timestamps, and STOP.
- Driver/encoder faults, explicit recovery, and absence of automatic restart.
- Nonblocking STATUS transmission: feedback continues while UART TXE paces
  output, transmission is bounded to 16 bytes per foreground call, and a blocked
  UART does not prevent stale-encoder shutdown.
- Live 20%, 50%, 100% duty updates preserve speed/position mode; invalid inputs
  are rejected and zero duty blocks START/MOVE/POS until explicitly restored.
- Original DC/SERVO operation and STATUS-only Hall diagnostics.

`results.json` includes source hashes and evidence; `build.log` records compiler
commands. Tests use the same ARMCC and Python/Unicorn dependencies as the other
verification suites. Generated objects, ELF files and reports are test artifacts.

These are software integration tests. BLDC and peripherals are mocked here;
physical current, PWM waveforms, real serial transport, rotor synchronization,
gearbox dynamics and achieved positioning accuracy are not verified.

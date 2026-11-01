# MOE 同步读人工回归 fixture

此目录仅供 `run_bldc_tests.py` 的 `synthetic_moe_readback_delay_old_vs_current` 对照测试使用，**不是当前生产固件或可刷机源码**。请勿将此目录加入 Keil 生产工程。

`bldc_motor.c` 保留修复前“写 MOE 后立即读回”的实现，配套旧头文件和旧测试 harness。测试使用独立 ARMCC/Unicorn 构建产物，不覆盖正常固件。MOE 读取延迟由明确人工模型注入；结果不是 STM32 周期精确模拟，也不证明实板故障根因。

以下四个文件在 2026-11-01 原样复制，未修改内容。表中的历史备份路径仅记载来源，运行测试**不需要**该备份目录；所有对照输入均已包含在本 fixture 中。测试外设边界头文件复用 `../../mock/`。

| 文件 | 复制来源（相对于固件工程根目录） | SHA256 |
|---|---|---|
| `bldc_motor.c` | `usar/backup/pre_bkin_diagnostic_20261101_133703/usar/bldc_motor.c` | `cfdb93fb92ce72ed1fd2d704b63f5a70059e4985e1733898d5d7423dcd878e36` |
| `bldc_motor.h` | `usar/backup/pre_bkin_diagnostic_20261101_133703/usar/bldc_motor.h` | `ed2279eb67f2935a742224c8c0b05ae74dc9a7b6fac23db8b9a4cb397e4995cd` |
| `harness.c` | `usar/backup/pre_bkin_diagnostic_20261101_133703/usar/verification/bldc_test/harness.c` | `91144c8aa6fa5e0fc69cd7a6e096432c8766a1de21455955a2440091870ec320` |
| `motor_config.h` | `usar/motor_config.h` | `10a16fc05274c69ccf8665a5cebffc1a84024ec612c0caf4133f083c74ed8c1d` |

`motor_config.h` 来自复制时的当前正常生产配置，固定本对照的宏参数。旧源文件与旧 harness 来自 `pre_bkin_diagnostic_20261101_133703` 快照。文件内部已有的来源及版权说明保持原样。

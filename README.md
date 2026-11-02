# STM32F103 六步电机驱动

当前工程位于 `stm32f103motor/stm32motor`，包含 STM32CubeMX 配置、Keil 工程、HAL/CMSIS 依赖、业务源码及可重放的软件回归测试。

## 功能

- BLDC 六步正反转、RS485 编码器反馈的输出轴转速控制、相对角度和绝对位置控制；保留原 CAN/USART3 占空比控制与霍尔模式。
- PB12 / TIM1_BKIN 低有效硬件保护、故障锁存及 `BKIN` 诊断；没有旁路开关。
- 母线电压与 PA2 双通道 ADC DMA 采样。
- PA4/PA5 方向控制、PB1 PWM 的有刷直流电机控制。
- PB8 舵机输出、USART1 RS485 与 MT_6V10 Modbus 编码器反馈。

霍尔接线：PA6=HA、PA7=HB、PB0=HC；当前编码为 `hall = (HC << 2) | (HB << 1) | HA`。电机实际霍尔/相线关系仍待实板验证，不应把软件回归通过视作整板或电机运行验收。

## 输出轴控制

默认 `MOTOR_OPEN_LOOP_TEST=1`。霍尔只用于诊断；编码器安装在减速箱输出轴，速度和位置指令也使用输出轴单位。控制器以实测“5 ms 换相约对应 30 rpm”为初始比例，用编码器反馈调整换相间隔，带加减速和停稳后反转。该比例不是已确认的减速比；转子电角度仍未闭环。已解除原 10% 占空比上限，`MOTOR SPEED` 接受 0 或 8..100%；上电默认仍为 10%，不会自动升至全占空比。原保护保留。

USART3：115200、8N1，每条命令以回车或换行结束。先按 30 rpm 验证：

```text
MOTOR STOP
MOTOR RPM 30
MOTOR DIR FWD
MOTOR START
STATUS
```

反转发送 `MOTOR DIR REV`；停止发送 `MOTOR STOP`。转速指令接受 `0`（停止）或 `3..350` rpm；350 rpm 是设定范围上限，实际能否达到需要实测。

位置控制先停机、等输出轴停稳，再设当前位置为零：

```text
MOTOR STOP
MOTOR ZERO
MOTOR RPM 30
MOTOR MOVE 90
```

`MOTOR MOVE 90` 立即从当前位置正向转动 90°；`MOTOR MOVE -90` 为反向相对转动；`MOTOR POS 0` 返回软件零点。角度为带符号整数度，支持多圈，目标范围 ±360000°。靠近目标时减速，到位后关闭 PWM，没有持续保持力矩。初始到位容差 2°，实际精度受减速箱间隙、惯性和编码器刷新速度影响，尚需上机验证。软件零点不写入 Flash，重启后重新建立。

`MOTOR SPEED 8..100` 设置当前模式占空比，不是 rpm；在 RPM/位置模式中修改占空比不会退出闭环或恢复固定换相。原固定 5 ms 模式仍保留，停止后切换 `MODE CAN` 再 `MODE UART` 可返回该模式。DC、SERVO 和 CAN 指令保留。详细命令、状态与故障处理见[业务目录说明](stm32f103motor/stm32motor/usar/README.md)。

## 构建

使用 Keil MDK / ARM Compiler 5 打开：

```text
stm32f103motor/stm32motor/MDK-ARM/stm32motor.uvprojx
```

也可以在 PowerShell 中从工程目录运行：

```powershell
cd .\stm32f103motor\stm32motor
powershell -ExecutionPolicy Bypass -File .\usar\verification\rebuild.ps1
```

构建脚本的默认 Keil 路径可通过 `-Uv4` 参数覆盖。HEX 由构建生成在 `MDK-ARM/stm32motor/stm32motor.hex`，不作为源码提交。

## 文档与测试

- [引脚、通信命令与保护说明](stm32f103motor/stm32motor/usar/README.md)
- [BLDC 回归测试](stm32f103motor/stm32motor/usar/verification/bldc_test/README.md)
- [定时六步测试](stm32f103motor/stm32motor/usar/verification/open_loop_test/README.md)
- [输出轴速度与位置测试](stm32f103motor/stm32motor/usar/verification/motion_test/README.md)
- [有刷电机回归测试](stm32f103motor/stm32motor/usar/verification/brushed_test/README.md)
- [CAN 协议回归测试](stm32f103motor/stm32motor/usar/verification/can_test/README.md)
- [MT_6V10 回归测试](stm32f103motor/stm32motor/usar/verification/mt6v10_test/README.md)

测试所需的 Python 依赖与 ARMCC 配置见各测试目录说明。日志、结果 JSON、编译中间文件及本地历史备份不纳入版本控制；必要的 MOE 同步读回归对照源独立保存在测试 `fixtures` 目录中。

HAL/CMSIS 与原工程代码的版权声明保留在对应文件中。

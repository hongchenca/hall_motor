# STM32F103 霍尔六步电机驱动

当前工程位于 `stm32f103motor/stm32motor`，包含 STM32CubeMX 配置、Keil 工程、HAL/CMSIS 依赖、业务源码及可重放的软件回归测试。

## 功能

- BLDC 霍尔六步换相，支持 CAN 或 USART3 开环占空比指令。
- PB12 / TIM1_BKIN 低有效硬件保护、故障锁存及 `BKIN` 诊断；没有旁路开关。
- 母线电压与 PA2 双通道 ADC DMA 采样。
- PA4/PA5 方向控制、PB1 PWM 的有刷直流电机控制。
- PB8 舵机输出、USART1 RS485 与 MT_6V10 Modbus 编码器反馈。

霍尔接线：PA6=HA、PA7=HB、PB0=HC；当前编码为 `hall = (HC << 2) | (HB << 1) | HA`。电机实际霍尔/相线关系仍待实板验证，不应把软件回归通过视作整板或电机运行验收。

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
- [有刷电机回归测试](stm32f103motor/stm32motor/usar/verification/brushed_test/README.md)
- [CAN 协议回归测试](stm32f103motor/stm32motor/usar/verification/can_test/README.md)
- [MT_6V10 回归测试](stm32f103motor/stm32motor/usar/verification/mt6v10_test/README.md)

测试所需的 Python 依赖与 ARMCC 配置见各测试目录说明。日志、结果 JSON、编译中间文件及本地历史备份不纳入版本控制；必要的 MOE 同步读回归对照源独立保存在测试 `fixtures` 目录中。

HAL/CMSIS 与原工程代码的版权声明保留在对应文件中。

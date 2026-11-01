# 有刷直流电机输出测试

该测试用 ARMCC 编译真实的 `usar/actuators.c`，并在 Unicorn 中执行 Cortex-M3 Thumb 代码。仅 HAL 调用、GPIO 和 TIM 寄存器存储由测试桩模拟；没有复制有刷电机控制算法。

在固件工程根目录（`stm32f103motor/stm32motor`）的 PowerShell 中运行，确保 Python 已加入 PATH：

```powershell
python .\usar\verification\brushed_test\run_brushed_tests.py
```

依赖 `D:\Keil_v5\ARM\ARMCC\bin` 下的 `armcc.exe`、`armlink.exe`，以及 `%TEMP%\stm32motor-can-test-python` 中的 `unicorn`、`pyelftools`。可使用 `ARMCC_BIN` 和 `STM32_TEST_PYTHON_DEPS` 覆盖默认路径。每次运行都会重新编译；全部通过时返回 0，否则返回 1。

覆盖：初始化默认停止；正向 PA4 高、PA5 低，反向 PA4 低、PA5 高；-1000..1000 permille 到 TIM3_CH4 CCR 的限幅/换算；0 速度和显式停止；反向切换期间先将 CCR4 置零、产生更新事件，再写方向脚；PWM 启动失败后 API 保持无输出。

`results.json` 记录模块 SHA256、编译命令、5 个用例状态和寄存器快照；`build.log` 保存编译与链接输出。该模拟不代替示波器测量、驱动器输入极性确认、真实电机启转/反转与功率级换向瞬态验证。

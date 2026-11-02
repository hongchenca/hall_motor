# 输出轴运动控制回归

在仓库根目录执行：

```powershell
py stm32f103motor/stm32motor/usar/verification/motion_test/run_motion_tests.py
```

使用 Keil ARMCC 编译实际 `motor_motion.c`，在 Unicorn 中执行。输入仅替代编码器数据和底层驱动就绪/故障状态，不复制控制算法。检查正负方向、输出轴多圈位置与编码器计数回绕、速度斜坡与反馈修正、停稳后反转、反馈过期/跳变/故障锁定、位置到达与越界纠偏、取消命令和重新设零。

测试中的理想减速轴模型只验证软件行为；实际可达速度、转矩、失步、减速箱惯性/间隙和定位精度仍需上机验证。生成的 `results.json` 记录源文件散列和每项证据。

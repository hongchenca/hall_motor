# STM32F103C8T6 电机工程移植

业务代码位于本目录，已接入 `Core/Src/main.c` 和 Keil 工程 `MDK-ARM/stm32motor.uvprojx`。打开该 Keil 工程即可编译，不需要再复制源文件。

## 已移植的内容

| 文件 | 内容与来源 |
| --- | --- |
| `bldc_motor.c/h` | 从原工程 `Source/Bldc.c`、`Protect.c`、`Include/Define.h` 移植霍尔六步换相、启动、母线采样、保护和堵转检测；重写 F103 ADC 与定时器部分 |
| `motor_config.h` | CAN 节点号、母线分压换算、保护阈值、20 kHz 控制时序 |
| `can_bus.c/h` | 按参考工程 `stm32-foc/software/Src/main.c` 的 CAN 协议移植过滤、接收和状态发送，补充帧校验、FIFO 清空、发送邮箱检查和未初始化字节清零 |
| `motor_app.c/h` | 主循环、故障灯、调试串口、CAN 指令到六步 PWM 的转换、有刷电机串口控制、TIM1 更新向量和 Break 回调 |
| `actuators.c/h` | 新增舵机与有刷电机基础 API |
| `rs485_port.c/h` | USART1（PB6/PB7，经重映射）原始字节收发和 PB5 方向控制 |
| `mt6v10_modbus.c/h` | MT_6V10 Modbus RTU 主站轮询、CRC/响应解析和角度/速度反馈 |

保留霍尔六步驱动及电压指令控制，当前默认构建为下面的定时开环测试版。原工程没有位置环、速度环；本次没有移植参考工程的 FOC 算法或虚构编码器数据。PA0 电位器输入未启用；PA2 已加入 ADC 双通道 DMA，当前仅提供原始采样值，具体外部信号与换算参数尚未确定。原 `IbusValue` 没有有效赋值，因此没有启用软件电流环或软件过流判断。PB12 / TIM1_BKIN 低有效硬件保护已恢复，旁路开关和旁路代码已移除。

## 定时开环六步

当前 `motor_config.h` 默认 `MOTOR_OPEN_LOOP_TEST=1`；设为 `0` 并重新构建、烧录可恢复原霍尔换相。开机串口显示 `OPEN_LOOP_TEST`，`STATUS/?` 增加 `commutation=OPEN_LOOP_TEST hall_used=0`。**这只是时间驱动的开环换相，不是反电动势闭环无感控制，也不是速度或电流闭环。**

- PA6/PA7/PB0 只提供诊断，不决定换相；0、7、不变化或跳变都不会触发本模式的霍尔故障。清故障不要求霍尔有效，其余保护条件不变。
- 收到 `MOTOR START` 后立即进入第 1 步，随后每 100 ms 换到下一步；每一步保持 100 ms，没有额外预充、对齐或加速斜坡。停止命令立即关闭输出。这是指令换相频率，不表示转子已跟随。
- 下表为实际励磁步序；FWD 按 1→2→3→4→5→6→1，REV 按 1→6→5→4→3→2→1。与实际机械旋向的对应关系仍需确认。

| 步号 | 高桥 PWM | 常导通低桥 |
| --- | --- | --- |
| 1 | U | V |
| 2 | U | W |
| 3 | V | W |
| 4 | V | U |
| 5 | W | U |
| 6 | W | V |

- UART 默认占空比为 10%，仍可用 `MOTOR SPEED 8..10` 调整；底层对 UART、CAN 的请求都限制到 10%（320/3200）。`SPEED` 是占空比，不调节换相周期，也不是 rpm。限占空比不等于限电流，PA2 尚无有效电流标定。
- 运行期间 USART3 每 100 ms 打印 `HALL=n`。霍尔值只作诊断，不参与换相。
- PB12 低有效 BKIN、Break 快照、母线过欠压、ADC 超时/DMA 错误和 500 ms 指令超时保护保留。该模式没有基于霍尔的堵转检测。

建议空载、使用限流电源短测，先检查驱动实际输出。USART3 每条命令后发送回车或换行：

```text
DC STOP
MOTOR STOP
MOTOR START
```

需要停止时发送 `MOTOR STOP`。`commutation` 行的 `step` 是上表 1～6，停止时为 0；`interval_us` 固定为 100000。未接霍尔时的 `hall=0/7` 在此模式仅是诊断，不代表换相失败。

## 当前引脚与时序

| 功能 | 引脚/配置 |
| --- | --- |
| 母线电压 | PA1 / ADC1_IN1，Rank 1，55.5 cycles，软件触发；20 kHz 采样，64 点平均 |
| 新增模拟采样 | PA2 / ADC1_IN2，Rank 2，55.5 cycles；保存最新原始值 0～4095 |
| 故障 LED | PA3，高电平亮；按故障码闪烁，250 ms 明灭、组间约 2 s |
| 霍尔 | **PB0 位 2、PA7 位 1、PA6 位 0**；测试版仅诊断，恢复霍尔模式后三次相同才接受 |
| BLDC U/V/W 高桥 | PA8 / PA9 / PA10，TIM1_CH1/2/3 |
| BLDC U/V/W 低桥 | PB13 / PB14 / PB15，TIM1_CH1N/2N/3N |
| 硬件故障 | PB12 / TIM1_BKIN 低有效，BKE 启用，AOE 关闭，故障锁存 |
| 有刷电机 | PA4 IN1、PA5 IN2、PB1 TIM3_CH4，20 kHz |
| 舵机 | PB8 TIM4_CH3，50 Hz，初始 1500 µs |
| CAN | PA11 RX、PA12 TX，1 Mbps，标准数据帧 |
| 调试/控制 USART3 | PB10 TX、PB11 RX，115200 8N1 |
| 编码器 RS485 | USART1 重映射：PB6 TX→SP485 DI、PB7 RX←SP485 RO；PB5 同时接 DE 和 /RE，高发送、低接收；38400 8N1 |
| 下载接口 | PA13 / PA14 保留 SWD |

PA0、PB2/BOOT1 和 BOOT0 未加入业务配置。使用内部 HSI：SYSCLK=64 MHz、APB1=32 MHz、定时器=64 MHz。CAN 分频 2、BS1=11TQ、BS2=4TQ、SJW=1TQ，采样点 75%。保持当前 CubeMX 定时器配置：TIM1/TIM3 PSC=0、ARR=3199；TIM2 PSC=63、ARR=65535；TIM4 PSC=63、ARR=19999。

ADC 时钟为 APB2/6，约 10.67 MHz。扫描顺序固定为 PA1→PA2，DMA1 Channel1 使用 Circular、半字宽度、内存地址递增。`MX_DMA_Init()` 在 `MX_ADC1_Init()` 之前执行。`Bldc_Init()` 校准后启动双元素 DMA；每次完整传输回调置位就绪标志，下一次 FastTick 读取两路结果后软件触发下一组，保持 20 kHz 调度。**Continuous Conversion 必须保持 Disabled**，防止 DMA 覆盖正在读取的数据；应用不再读取 ADC DR/EOC。半传输中断在启动后关闭，只处理完整两路数据。

TIM1 Update 每 50 µs 执行一次电机逻辑，优先级 2；Break 优先级 0、CAN RX0 优先级 1。`TIM1_UP_IRQHandler` 由 `motor_app.c` 提供，不要再生成另一份同名向量。Break 使用生成代码中的 HAL 入口和本模块的 `HAL_TIMEx_BreakCallback`；不要把 `Bldc_FastTick()` 放进共享的 `HAL_TIM_PeriodElapsedCallback()`，否则 Break 路径可能重入它。Break 故障锁存后关闭 BIE，满足条件并手动清故障后重新启用 BIE；硬件 BKE 保持启用。TIM2 专供微秒计时，不能挪作其他用途。

## CAN 协议与操作

默认节点号为 `motor_config.h` 的 `MOTOR_CAN_NODE_ID=1`，有效范围 1～8，当前通过重新编译修改，不读写 Flash 节点号。

控制帧要求：**标准帧、数据帧、DLC=8**。节点 1～4 接收 ID `0x100`，节点 5～8 接收 ID `0x200`。每两个字节为一个节点的小端 `int16_t` 毫伏指令：

| 字节 | 0x100 | 0x200 |
| --- | --- | --- |
| 0、1 | 节点 1 | 节点 5 |
| 2、3 | 节点 2 | 节点 6 |
| 4、5 | 节点 3 | 节点 7 |
| 6、7 | 节点 4 | 节点 8 |

例如节点 1：ID `0x100`、数据 `88 13 00 00 00 00 00 00` 表示 +5000 mV；`78 EC 00 00 00 00 00 00` 表示 -5000 mV；全零表示停止。应持续周期发送，例如每 10 ms 一帧。指令按母线电压换算为占空比，低于 8% 会停止；当前开环模式上限为 10%，恢复霍尔模式后上限为 100%。负值在开环模式中反向遍历定时六步表，在霍尔模式中使用霍尔反向映射。方向改变前会关闭输出等待 100 ms；这段固定等待不代表已检测到转子停止。

CAN 控制模式下，500 ms 没有有效控制帧会自动关闭 BLDC 输出。计时使用 CAN 接收时刻，主循环反复执行不会刷新期限。串口电机命令会切换为 UART 控制模式；此模式下串口目标保持有效，直到显式停止或切回 CAN。CAN 协议只控制 BLDC；有刷电机和舵机没有分配额外 CAN 命令。

反馈 ID=`0x100+节点号`，DLC=8，每 2 ms 最多尝试发送一次，邮箱满时跳过：

| 字节 | 数据 |
| --- | --- |
| 0～3 | 小端 `int32_t`，机械角度 rad × 1000，即 mrad |
| 4～5 | 小端 `int16_t`，机械转速 rpm × 10 |
| 6～7 | 0 |

角度和转速反馈来自 MT_6V10 编码器的有效 Modbus 样本；未收到有效样本前 CAN 反馈保持 0。没有额外施加极对数或机械零点假设。

## 调试与故障恢复

调试与控制串口使用 USART3（PB10/PB11），115200 8N1。文本命令以回车或换行结束，不区分大小写；`?` 和 `c` 仍可作为单字符快捷键。USART1 专用于 RS485，38400 8N1，避免控制台输出与收发器共享 UART。

| 命令 | 作用 |
| --- | --- |
| `HELP` | 列出命令 |
| `STATUS` 或 `?` | 输出控制源、BLDC 状态/故障、母线电压、PA1 平均值 `adc`、PA2 原始值 `pa2`、占空比、霍尔状态，以及编码器在线状态、角度/速度、原始角度、多圈计数、CRC 错误、响应超时和 USART1 接收溢出数 |
| `BKIN` | 只读输出 PB12/定时器当前状态，以及首次 `fault=5` 的来源和寄存器快照；`STATUS/?` 也追加这两行 |
| `MOTOR SPEED 8..10` | 设置 BLDC 占空比百分比，不会自动启动；当前默认为 10，恢复霍尔模式后范围为 8..100 |
| `MOTOR SPEED 0` | 清除速度设定并停止 BLDC |
| `MOTOR START` / `MOTOR STOP` | 以当前速度启动/停止 BLDC；默认速度为 10% 占空比 |
| `MOTOR DIR FWD` / `MOTOR DIR REV` | 设置无刷换相方向；运行中换向会先滑行约 100 ms。FWD/REV 与实际电机旋向的对应关系需实机确认 |
| `DC SPEED 0..100` | 设置有刷直流电机 PWM 百分比；不会启动电机，0 会立即停机 |
| `DC DIR FWD` / `DC DIR REV` | 设置有刷电机方向并先停机；FWD=PA4 高/PA5 低，REV=PA4 低/PA5 高，方向实际对应关系需实机确认 |
| `DC START` / `DC STOP` | 启动/停止有刷电机；启动前先设置非零速度，停止时 PB1 PWM 清零且 PA4/PA5 均拉低 |
| `SERVO ANGLE -60..60`（或 `SERVO -60..60`） | KST A08，按 900..2100 µs 线性设置 PB8 舵机脉宽，0° 为 1500 µs |
| `MODE UART` / `MODE CAN` | 切换控制源。任一 `MOTOR` 命令会自动切到 UART；切到 CAN 会停止串口目标并等待新的 CAN 指令 |
| `FAULT CLEAR` 或 `c` | 尝试清除锁存故障；须先停止电机，母线正常、PB12 为高；仅霍尔模式还要求霍尔 1～6；ADC DMA 错误须复位恢复 |

霍尔码按 `PB0:PA7:PA6` 拼接：`hall_raw = 4×PB0 + 2×PA7 + PA6`。测试霍尔时保持驱动停止，手动缓慢转动电机轴；从 USART3 发送 `STATUS` 或 `?` 查看原始值。运行期间，开环模式每 100 ms 自动打印 `HALL=n`。完整电周期应出现 6 个有效霍尔码 1～6，相邻状态通常只改变一位；0 和 7 对原霍尔算法无效。开环模式的霍尔值仅用于诊断，不参与换相。

`MOTOR SPEED` 是无刷电机的开环电压指令百分比，不是 rpm，也没有速度闭环；编码器反馈仅供 CAN 测量回传，不参与电机控制。直流电机使用独立的 `DC` 命令，不受 BLDC 的 CAN/UART 控制源切换影响。DC 方向命令会先停机，切向后必须再次发 `DC START`；每个控制命令无需周期刷新。KST A08 舵机脉宽按已提供规格限制为 900..2100 µs，对应 -60..60°。

无刷电机发送 `MOTOR START` 启动、`MOTOR STOP` 停止；可选发送 `MOTOR SPEED 8..10` 或 `MOTOR DIR FWD|REV`。有刷电机例如发 `DC SPEED 60`、`DC DIR FWD`、`DC START`；停止发 `DC STOP`，反转发 `DC DIR REV` 后重新 `DC START`。舵机中位发 `SERVO ANGLE 0`，两端发 `SERVO ANGLE -60` 和 `SERVO ANGLE 60`。每条命令后发送 CR、LF 或 CRLF。

PA2 尚未换算为电流或外部信号电压。旧的单字符 `s` 停止快捷键改用 `MOTOR STOP`；该命令只停止 BLDC，舵机保持。

状态码：0 停止、1 霍尔模式启动预充、2 运行、3 换向等待。故障码：1 霍尔无效或约 3.28 s 无变化（仅霍尔模式）；2 欠压；3 过压；5 BKIN 硬件故障；6 ADC 超时或 DMA 错误。LED 闪相应次数。故障不会因母线恢复或下一帧非零命令自行解除。ADC 超时后需要重新收齐 64 组新数据才恢复电压就绪；DMA 传输错误会立即关闭 BLDC 输出，禁止通过 `c` 清除，需复位重新初始化。

验证 PA2 时，在不发送电机运行指令的情况下，使用 USART3（PB10/PB11）的 115200 8N1 发送 `?`，观察 `pa2`。PA2 接地应接近 0，接入相对 VDDA 的已知模拟电压后应接近 `Vin / VDDA × 4095`。保持 PA1 输入不变，改变 PA2 时，`adc` 与 `vbus` 应保持在原有噪声范围内。

主程序初始化失败、CPU fault 或 `Error_Handler()` 会关闭 BLDC、有刷和舵机输出，需要复位后恢复。正常零命令、CAN 超时或串口 `MOTOR STOP` 只停止 BLDC；有刷电机用 `DC STOP` 独立停止。

## PB12 / TIM1_BKIN 过流保护

PB12 正常为高电平，外部过流电路触发时拉低。TIM1 保持 `BreakState=TIM_BREAK_ENABLE`、`BreakPolarity=TIM_BREAKPOLARITY_LOW`、`AutomaticOutput=TIM_AUTOMATICOUTPUT_DISABLE`。低电平触发硬件关断，并由软件锁存 `fault=5`；即使信号随后恢复高电平，也不会自动恢复输出。

故障排除后，先发 `MOTOR STOP`，确认 PB12 为高电平、母线正常，再发 `FAULT CLEAR`；只有霍尔模式还要求霍尔正常。清故障后 BLDC 保持停止，需要新的启动命令。旁路宏、绕过 PB12/BIF 的分支及串口旁路标志已删除；重新烧录当前 HEX 后才会在实板生效。PB12 不使用内部上下拉，外部电路必须提供确定电平。

### 启动闪 5 次、PB12 测量为高时的诊断

`fault=5` 不只来自读到 PB12 低，还可能来自 Break 中断、残留 BIF 或输出使能检查失败，不等于已经测得实际过流。RM0008 的高级定时器 Break 章节要求 MOE 从 0 写 1 后延迟再读回，因为写入和读回同步方式不同；当前已在写 MOE 后增加现有 TIM2 的 1 µs 等待，再检查 MOE/PB12/BIF。等待期间不关闭 BKE、不清除 BIF。

`BKIN live` 是查询时的状态。当前无重映射的正常配置应为 `bke=1 bkp=0 map=0`；PB12 空闲应为 `pb12=1`。`moe` 表示当前输出使能，`bif` 为当前 Break 标志，`bie` 为 Break 中断使能，`css` 为时钟安全系统开关。故障锁存后 `moe=0`，Break IRQ 后 `bie=0` 都可能是正常保护动作。

`BKIN last` 保存第一次 `fault=5` 的来源，只有初始化或**成功**执行 `FAULT CLEAR` 才会清除；`MOTOR STOP`、查询和失败的清故障不会抹掉记录。`sr/bdtr/dier` 为十六进制寄存器值。

| `src` | 含义 |
| --- | --- |
| `NONE` | 尚无已记录的 Break 故障，或刚成功清除 |
| `INIT_LOW` | 初始化检查读到 PB12 低 |
| `TICK_LOW` | 控制周期检查读到 PB12 低 |
| `IRQ` | 收到 Break 中断；在 HAL 清 BIF 前抓取现场 |
| `START_GUARD` | 写 MOE 前的故障/PB12/BIF 条件不满足 |
| `START_VERIFY` | 写 MOE 并等待后，MOE/PB12/BIF 复查失败 |

发生故障后先发 `BKIN` 或 `?` 保存输出，**不要先清故障**。万用表读数和停机后的 `pb12=1` 不能排除启动瞬间的低脉冲；若 `src=IRQ`，即使实时 `bif=0` 也不能否认硬件事件，因为 HAL 会先清标志再进入回调。快照是顺序读取的软件记录，不能代替示波器。

原理图/PCB 网络显示 PB12 由 R19=3kΩ 上拉至 3.3 V，Q3 集电极经 R21=510Ω 接 PB12，PB12 侧 C19=1nF 对地；U8 输出经 R18 驱动 Q3。因此电路意图确实为低有效，不应因测到 3.2 V 就改成高有效。实板仍需核对：示波器 CH1 测 PB12（R21 后），CH2 测 U8 pin4 OUT，在启动前用 PB12 下降沿单次触发布防，并使用合适的地参考。上述网络核对不代表已验证焊接和实际波形。

## 扩展模块 API

在 `MotorApp_Init()` 成功后，从主循环调用：

```c
#include "actuators.h"
#include "rs485_port.h"

Servo_SetPulseUs(1500);       /* KST A08: 900..2100 us, center = 1500 us */
BrushedMotor_SetDuty(300);    /* -1000..1000，例为正向 30% */
BrushedMotor_SetDuty(0);      /* PWM=0，两个方向脚均低 */
Actuators_Stop();            /* 仅停止有刷，舵机保持 */
```

有刷换向先清零并立即提交 PWM 预装载，再更改方向引脚。RS485 提供 `RS485_Transmit(data,length,timeout_ms)`、`RS485_Receive(data,length,timeout_ms)` 原始字节接口；发送等待 TC 后回到接收模式。SP485EEN-L/TR 的 DI 接 PB6（USART1_TX）、RO 接 PB7（USART1_RX），PB5 同时控制 DE 和 /RE，高电平发送、低电平接收。MT_6V10 Modbus 轮询由 `mt6v10_modbus.c` 在 `MotorApp_Process()` 中执行，不要从其他模块并发调用 USART1。

## MT_6V10 Modbus RTU

USART1 使用默认站号 1、38400 8N1。主循环每 20 ms 使用 FC03 从寄存器 1 连续读取两个寄存器：寄存器 1 为单圈角度原始值，寄存器 2 为有符号多圈数。寄存器内容高字节先发，RTU CRC-16/Modbus 低字节先发。收到完整且 CRC 正确的应答后，角度换算为连续 mrad；相邻位置样本计算转速并以 0.1 rpm 为单位回传 CAN。响应由主循环逐字节轮询，不等待整帧；20 ms 无响应记超时，100 ms 无有效样本置离线并将速度清零。

USART1 使用单字节接收中断写入 63 字节环形缓冲，供主循环解析，避免控制台输出阻塞期间丢失编码器应答。通过 USART3 发送 `STATUS` 或 `?` 检查 `enc=online`、`angle`、`rpm_x10`、`raw`、`turns`、`crc`、`timeout` 和 `rx_ovr`。当前尚无 RS485 实板验证；若编码器站号被改写或串口参数被配置过，需先按实际设备设置调整。

## 硬件参数与验证边界

母线阈值沿用原板：ADC≤1805 约 16 V 欠压，ADC>3385 约 30 V 过压，持续约 10 ms 锁存。电压换算采用 ADC=3385 对应 30000 mV，假定 VDDA=3.3 V 及原分压网络。若 F103 板分压不同，必须修改 `motor_config.h`，否则保护和电压指令到占空比的换算会不准确。

六路驱动逻辑沿用原板：高、低桥驱动输入高有效；霍尔模式要求 U/V/W 与霍尔线序相符。CCER 全关期间不能仅依靠 OSSI/OSSR 保证六路都驱动为低，门极驱动输入应有有效下拉。上板应核对停机、Break 和换相间隙的实际输出电平。当前不使用同相上下桥互补 PWM，也没有新增适用于未知功率级的死区参数。霍尔无内部上下拉，依赖外部电路提供确定电平；BKIN 也需要外部确定电平。

目前已做软件构建与逻辑验证，尚未连接硬件测试。内部 HSI 在 1 Mbps CAN 下的误差、实际总线误码、母线 ADC 标定、驱动极性、霍尔/相线顺序和电机反转行为需要实板核对。

## 构建、重生成与备份

2026-11-01 的定时开环测试版全量 Keil 构建为 **0 错误、0 警告**；Code=23944、RO=1612、RW=116、ZI=2444 字节。默认模式独立 ARM 仿真 **18/18** 通过，另以 `MOTOR_OPEN_LOOP_TEST=0` 重跑原霍尔回归 **25/25** 通过。有刷 **5/5**、CAN **507 条断言**、编码器 **41 条断言**也通过。测试方法和硬件边界见 `verification/open_loop_test/README.md`，未烧录或实测电机。

当前烧录文件为 `MDK-ARM/stm32motor/stm32motor.hex`，另保存同内容的明确命名副本 `stm32motor_open_loop_test.hex`。正常霍尔版源码与 HEX 已在 `backup/pre_open_loop_test_20261101_145358` 保存；恢复时也可将模式宏改为 0 后重新编译。备份与 HEX 都不纳入 Git 源码。

当前串口命令版本已用 Keil ARM Compiler 5.06 update 7 build 960 完整编译，**0 错误、0 警告**；Keil 输出日志为 `MDK-ARM/stm32motor/stm32motor.build_log.htm`，固件输出在 `MDK-ARM/stm32motor/stm32motor.hex`。可在工程根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\usar\verification\rebuild.ps1
```

UART 句柄路由为 USART3 控制台 PB10/PB11、USART1 RS485 PB6/PB7、PB5 控制并联的 DE 和 /RE。霍尔逻辑码和 `hall_raw` 按 PB0:PA7:PA6 位序，仿真同步注入对应物理引脚。模式 0 的 **25/25** 霍尔回归保留低有效保护、首次故障快照、MOE 等待期间短 Break 及人工同步读对照；该对照不是周期精确模型，也不证明实板故障根因。默认模式 1 的 **18/18** 开环回归独立验证双向六步、时间边界、占空上限、累计预算和保护。当前测试目标仍启用正常 BKIN 保护，RS485 尚未实板验证。复跑步骤见各 `verification/*_test/README.md`，源码与固件哈希、构建摘要见 `verification/port_validation.json`。模拟测试不代替实板验证。

主程序集成与 fatal 停机调用均位于 CubeMX USER CODE 区域。USART1 的 38400 波特率和 NVIC 优先级 6 已写入 `stm32motor.ioc`。重新生成后核对 Keil 仍包含 `../usar` 头文件路径及 `Application/User/usar` 组的六个 `.c` 文件。测试用的 mock/HAL 文件位于 `verification`，不加入固件构建。

修改前的配置、main、中断文件和 Keil 工程已备份在 `backup/pre_port_20261017_1502`。原 G030 工程与参考 FOC 工程未改动。

本次双通道 DMA 接入前的业务代码、验证文件及用户刚生成的 ADC/DMA 配置备份在 `backup/pre_adc_dma_20261017_194542`。

本次 UART 路由修改前的源码、说明、验证摘要和 HEX 固件备份在 `backup/pre_uart_route_20261029_163235`。

本次增加停止状态霍尔诊断输出前的源码、说明、验证摘要和 HEX 固件备份在 `backup/pre_hall_diagnostic_20261029_171044`。

本次霍尔逻辑码 bit 顺序调整前的 BLDC/状态输出代码、仿真输入、说明、结果摘要和 HEX 固件备份在 `backup/pre_hall_bit_reverse_20261029_173515`。

本次恢复霍尔拼接顺序为 PB0:PA7:PA6 前的源码、测试输入和说明备份在 `backup/pre_hall_order_restore_20261030_103109`。

本次临时启用 BKIN 旁路前的源码、测试文件、验证记录与 HEX 备份在 `backup/pre_bkin_bypass_test_20261030_111005`。

2026-11-01 删除旁路并恢复正常保护前的源码、测试文件、验证记录及固件备份在 `backup/pre_bkin_restore_20261101_113617`。该历史备份不参与当前构建，不要将其中的旧旁路 HEX 当作正常版烧录。

2026-11-01 增加 MOE 同步等待和 BKIN 故障来源诊断前的正常保护版备份在 `backup/pre_bkin_diagnostic_20261101_133703`。该备份用于人工同步模型的新旧源码对照，不是新的烧录目标。

本次新增有刷电机串口控制前的应用代码、验证文件、说明、摘要和 HEX 固件备份在对应的 `backup/pre_brushed_console_*` 目录。

有刷电机输出回归使用 `verification/brushed_test/run_brushed_tests.py` 编译并执行真实 `actuators.c`，覆盖启动默认停机、PA4/PA5 互斥、PWM 换算、零速/停止及先清 PWM 再换向。测试快照只验证软件写入顺序，不等同于实测引脚波形或驱动器瞬态；上板时先断开电机或限流供电，确认方向与 PWM 极性后再带载。

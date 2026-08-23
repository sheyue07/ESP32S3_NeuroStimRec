# ESP32-S3 复刻 FPGA 刺激控制功能——修改规格

## 1. 文档用途

本文档用于指导在下列 ESP-IDF 工程中直接实现原 FPGA
`StimRec64Ch.v` 的刺激控制功能：

- 目标工程：`E:\ESP_IDF_File\ESP32S3_30M_ADC_STREAM_TO_SDMMC`
- FPGA 参考代码：
  `E:\graduation_project\RS64_Code\Code_rs64_0822\30M_Code_rs64R_0104_true\StimRec64Ch.v`
- 目标芯片：ESP32-S3
- ESP-IDF：v6.0.2

本次只移植刺激时钟、刺激控制和 40 位配置帧发送功能。FPGA 文件只作为
行为参考，其中的无用分频器、未初始化寄存器、无效隐式网络和乱码注释不要
机械照搬。

## 2. 不可改变的现有功能

现有 ADC/SD 采集流程必须保持不变：

- GPIO20：外部 30 MHz ADC 采样时钟输入。
- GPIO16：ADC 串行数据输入。
- GPIO7：只控制是否把 ADC 数据写入 SD 卡；高电平开始记录，低电平停止。
- SPI2_HOST：现有 30 MHz 连续 ADC 接收和 GDMA，不得被刺激模块占用或重置。
- SDMMC：GPIO41/42/40/39/1/2，保持 4-bit、20 MHz 配置。
- 不修改 ADC 260 字节帧格式、帧同步算法、PSRAM 环形缓冲和裸 SD 分段格式。

刺激模块初始化、运行或故障都不能改变 GPIO7 的含义。

## 3. 新增引脚定义

| ESP32-S3 GPIO | 信号 | 方向 | 要求 |
|---|---|---|---|
| GPIO19 | `mclkST` | 输出 | 上电初始化成功后连续 6.6 MHz |
| GPIO8 | `SCLK` | 输出 | 连续 6.6 MHz，与 GPIO19 严格反相 |
| GPIO17 | `MOSI` | 输出 | 在 SCLK 下降沿附近更新，在上升沿前保持稳定 |
| GPIO15 | `CSb` | 输出 | 低有效；每个 40 位帧低 40 个时钟 |
| GPIO6 | `Stim_ctrl` | 输入 | 高电平请求启动刺激，低电平请求停止刺激 |

GPIO6 配置下拉，按 100 us 稳定时间消抖。不得把 GPIO6 和 GPIO7 合并。
原 FPGA 的 `FPGA1` 状态输出本次没有分配引脚，不实现外部输出；可在日志或
内部状态中保留等价的 `stim_enabled` 状态。

## 4. 选定实现方案

采用 **LCD_CAM 并行输出 + 循环 GDMA**，不用普通 SPI3 直接产生外部时序。

原因如下：

- FPGA 的 `mclkST` 与 `SCLK` 在 `CSb=1` 时仍持续运行。
- FPGA 中 `SCLK = ~mclkST`，两路必须同源并保持固定反相关系。
- 普通 ESP32 SPI 主机只在事务期间输出 SCLK，无法复刻帧间连续时钟。
- 使用 LCD_CAM 像素时钟作为唯一时基，GDMA 每个时钟输出一个并行数据样本，
  可以同时控制 MOSI 和 CSb，并避免 CPU 以 6.6 MHz 翻转 GPIO。

### 4.1 信号路由

- 将 LCD_CAM 的像素时钟路由到 GPIO8，作为 SCLK。
- 将同一个像素时钟通过 GPIO Matrix 反相后路由到 GPIO19，作为 `mclkST`。
- 将 LCD 数据位 D0 路由到 GPIO17，作为 MOSI。
- 将 LCD 数据位 D1 路由到 GPIO15，作为 CSb。
- 其余 LCD 数据位不连接外部 GPIO，也不得占用现有工程引脚。

每个 DMA 字节只使用低两位：

```text
bit0 = MOSI
bit1 = CSb
bit7:2 = 0
```

必须核对 ESP-IDF v6.0.2 和 ESP32-S3 的实际 LCD_CAM/GPIO Matrix 信号索引，
不能从其他 ESP32 型号复制寄存器编号。

### 4.2 6.6 MHz 时钟

优先选择 160 MHz PLL 作为 LCD_CAM 时钟源。理论分频关系为：

```text
160 MHz / (24 + 8/33) = 6.6 MHz
```

实施时必须按 ESP32-S3 LCD_CAM 时钟寄存器的真实分数分频公式配置，并从
实际寄存器值反算输出频率。若公开 `esp_lcd` API 无法建立无间隙循环 DMA，
允许参考现有 `continuous_rx.c` 的风格使用 ESP-IDF v6.0.2 的 LL/HAL 和 GDMA
接口，但要把底层访问封装在单独模块中。

不接受用 FreeRTOS 延时、GPIO bit-bang、GPTimer ISR 或软件循环产生 6.6 MHz。
也不允许在未报告的情况下退回“LEDC 主时钟 + 普通 SPI”方案。

## 5. 原 FPGA 的时序语义

FPGA 原代码执行以下关系：

```verilog
assign mclkST_In_FPGA = CLK_6MHz;
assign Testclk = CLK_6MHz;
assign SCLK_FPGA = ~Testclk;
```

因此 GPIO19 与 GPIO8必须连续运行且互为反相。FPGA 在 `Testclk` 上升沿改变
MOSI 和 CSb；这对应 SCLK 下降沿更新数据，刺激芯片在下一次 SCLK 上升沿
采样，等价于 MSB-first、接近 SPI Mode 0 的数据时序。

### 5.1 单帧波形

一个发送槽固定为 101 个 6.6 MHz 时钟：

- 样本 0..39：`CSb=0`，依次输出 40 位帧的 bit39..bit0。
- 样本 40..100：`CSb=1`，`MOSI=1`。

对应时间：

- CSb 低：40 / 6.6 MHz，约 6.061 us。
- CSb 高：61 / 6.6 MHz，约 9.242 us。
- 一个发送槽：101 / 6.6 MHz，约 15.303 us。

CSb 只能在 SCLK 的安全边沿改变。MOSI 必须在 SCLK 上升沿前稳定，并在
该上升沿后满足保持时间。

### 5.2 GPIO6 消抖

参考代码的 `CNT_MAX=660` 在 6.6 MHz 下实际约为 100 us，不是注释声称的
10 ms。本实现按实际行为使用 100 us：

- GPIO6发生变化后启动或重启一次 100 us 单次定时。
- 定时到期再次读取GPIO6；电平与候选状态一致才提交新状态。
- GPIO中断和定时回调只投递状态事件，不构造帧、不操作大块DMA链。
- DMA波形切换在101样本边界进行，避免输出半帧；相对FPGA的最大附加响应
  延迟小于一个发送槽，即约15.303 us。

## 6. 40 位协议格式

每帧5字节、40位、MSB first：

```text
Byte0 = {R/W, Global/Individual, Enable/Stop,
         STclk_Sel[1:0], Reserve[1:0], AddrReg[2]}
Byte1 = {AddrReg[1:0], AddrCh[5:0]}
Byte2 = Data[15:8]
Byte3 = Data[7:0]
Byte4 = (Byte0 + Byte1 + Byte2 + Byte3) & 0xFF
```

保持参考代码中的定义：

```text
Write=0, Read=1
Individual=0, Global=1
StopStim=0, EnStim=1
STclk_Sel=0
Reserve=0
AddrCh=35 (0x23)
```

不要把校验和写成异或或CRC。

### 6.1 启动序列的固定帧

GPIO6稳定变高后按以下顺序发送10帧。右侧给出应当生成的完整5字节结果，
可直接作为单元测试黄金值：

| 顺序 | FPGA帧名 | 寄存器/数据 | 5字节（十六进制） |
|---:|---|---|---|
| 0 | `FrameStop` | Global Stop, ID=`0x2A0D` | `40 23 2A 0D 9A` |
| 1 | `Frame0` | Read Reg0, ID=`0x2A0D` | `80 23 2A 0D DA` |
| 2 | `Frame1` | Reg1 Mode=`0x0006` | `00 63 00 06 69` |
| 3 | `Frame2` | Reg2 Freq=`0xC323` | `00 A3 C3 23 89` |
| 4 | `Frame3` | Reg3 PulseNum=`0x0064` | `00 E3 00 64 47` |
| 5 | `Frame4` | Reg4 PulseWA=`0x0014` | `01 23 00 14 38` |
| 6 | `Frame5` | Reg5 PulseGap=`0x0005` | `01 63 00 05 69` |
| 7 | `Frame6` | Reg6 PulseWC=`0x0014` | `01 A3 00 14 B8` |
| 8 | `Frame7` | Reg7 PulseAMP=`0xFFFF` | `01 E3 FF FF E2` |
| 9 | `Frame8` | Reg0 Stim=`0xFFFF`, Enable | `20 23 FF FF 41` |

这些值必须由统一的帧构造函数生成并与黄金值比较，不能仅硬编码字节后跳过
协议字段和校验和测试。

## 7. 刺激状态机

状态至少包含：

```text
STOP_LOOP -> START_SEQUENCE -> ENABLED_IDLE
```

### 7.1 上电及 GPIO6 低电平

- 在启动 LCD_CAM/GDMA 前先用普通 GPIO 将 CSb 和 MOSI置高，避免上电毛刺
  形成伪命令。
- GPIO6默认为下拉。
- 波形引擎启动后进入 `STOP_LOOP`。
- `STOP_LOOP` 循环发送 `FrameStop`：40个发送样本 + 61个空闲样本。
- 这与原 FPGA 在 `Stim_ctrl=0` 时不断发送停止帧的行为一致。

### 7.2 GPIO6 稳定变高

- 在下一个101样本边界切换到 `START_SEQUENCE`。
- 顺序发送第6.1节列出的10帧，每帧都有完整61周期帧间隔。
- 10帧共1010个样本，耗时约153.030 us。
- 发送完成后进入 `ENABLED_IDLE`。
- `ENABLED_IDLE` 中 CSb=1、MOSI=1，但两路6.6 MHz时钟继续运行。
- GPIO6保持高电平时不得重复发送启动序列。

### 7.3 GPIO6 稳定变低

- 在安全的101样本边界进入 `STOP_LOOP`。
- 随后持续循环发送停止帧。
- 清除“启动序列已发送”状态，使下一次稳定高电平可以再次发送10帧。

GPIO6在启动序列中途变低时，应在当前40位帧和所属101样本槽完成后切换到
`STOP_LOOP`，不得截断帧或产生窄CS脉冲。

## 8. DMA缓冲与模块边界

建议新增以下文件，具体名称可以保持等价但职责不能混在 `main.c` 中：

```text
main/stim_protocol.h
main/stim_protocol.c
main/stim_waveform.h
main/stim_waveform.c
main/stim_controller.h
main/stim_controller.c
```

- `stim_protocol`：字段定义、40位帧构造、8位加法校验和、10帧序列。
- `stim_waveform`：把帧转换成每样本D1/D0波形，管理LCD_CAM和循环GDMA。
- `stim_controller`：GPIO6配置、100 us消抖、状态转换和故障安全处理。
- `main.c`：只负责调用刺激模块初始化，不放入时钟寄存器和帧拼接细节。

建议预先构造三类内部 SRAM、DMA-capable 缓冲：

- `stop_loop[101]`：停止帧循环。
- `start_sequence[1010]`：10帧一次性序列。
- `enabled_idle[101]`：全部 `CSb=1, MOSI=1` 的空闲循环。

缓冲区不得放在PSRAM。描述符必须预分配，运行中不做动态内存申请。描述符
切换必须在EOF/槽边界完成，并保证像素时钟无停顿、LCD FIFO不下溢。

如果硬件要求8-bit LCD总线，可以仍只向GPIO Matrix连接D0和D1；禁止为了
满足总线宽度而随意占用六个额外外部GPIO。

## 9. 资源与并发约束

- 刺激波形使用 LCD_CAM TX 和独立 GDMA TX 通道。
- 不使用 SPI2_HOST；SPI2由 `continuous_rx` 独占。
- 不重新初始化 SDMMC 或现有 GDMA RX。
- 刺激连续输出由硬件和DMA维持，不能依赖FreeRTOS任务调度精度。
- 控制任务应事件驱动且低负载，不应降低现有 ADC DMA 任务优先级。
- 约1.2 KiB的波形数据和少量DMA描述符应从内部DMA内存分配。
- 在 ADC 30 MHz采集和SD写入同时运行时，不得出现新的DMA序号断裂、环形
  缓冲溢出、帧同步丢失或SD写入失败。

## 10. 故障安全

- 初始化任何一步失败：记录明确错误；CSb保持高、MOSI保持高，不发送启动
  帧；不得输出不完整帧。
- DMA FIFO下溢、描述符错误或意外停止：立即禁止刺激命令，尽可能将CSb和
  MOSI恢复为高电平，并报告错误计数。
- 刺激模块失败不应破坏ADC/SD模块的已有资源；ADC记录功能可以继续运行。
- 不允许静默采用近似时序或普通SPI作为回退。
- 日志至少包含初始化频率、所用时钟分频参数、GPIO6稳定状态转换、启动序列
  完成、停止状态和任何DMA故障。

## 11. 构建系统和文档

- 修改 `main/CMakeLists.txt`，加入新增源文件及实际使用的官方组件依赖。
- 优先使用ESP-IDF公开驱动；若为实现连续循环必须使用LL/HAL，依赖要精确
  写出，并把版本相关代码限制在 `stim_waveform.c`。
- 更新README的硬件引脚表、GPIO6/GPIO7职责和刺激状态说明。
- README必须说明 GPIO19和GPIO8是高速连续时钟，布线需短、共地，并根据
  实测波形选择合适驱动强度，避免默认直接设为最大驱动。

## 12. 验证要求

### 12.1 自动测试

至少覆盖：

1. 四字节求和只保留低8位。
2. 10个启动帧逐字节等于第6.1节黄金值。
3. 每帧恰好40个CS低样本和61个CS高样本。
4. 40位按MSB first映射到MOSI。
5. `stop_loop`为101样本，`start_sequence`为1010样本。
6. `enabled_idle`所有样本均为 `CSb=1, MOSI=1`。
7. GPIO6高状态不重复启动，低状态重新装载停止循环。
8. GPIO7相关源码和现有采集协议未被刺激模块改写。

构建命令：

```powershell
idf.py set-target esp32s3
idf.py build
```

### 12.2 示波器/逻辑分析仪

在只启动刺激模块和ADC/SD满负载两种情况下分别验证：

- GPIO8：6.600 MHz连续时钟，CS高时也不停止。
- GPIO19：6.600 MHz连续时钟，与GPIO8互为反相。
- 频率误差不超过板载时钟源精度和测量误差；建议验收范围 ±0.1%。
- 两路时钟占空比接近50%，无描述符边界缺脉冲或异常窄脉冲。
- GPIO15每帧低电平覆盖恰好40个SCLK周期，高电平覆盖61个周期。
- GPIO17在SCLK上升沿前稳定，逻辑分析结果与黄金帧一致。
- GPIO6低时重复停止帧；稳定高后只发送一次10帧序列；再次低后恢复停止帧。
- GPIO7只影响SD记录，不触发或停止刺激。

### 12.3 系统回归

- 使用可覆盖数据的测试SD卡运行，注意当前工程会覆盖裸卡LBA区域。
- 同时开启ADC采集、刺激和SD记录，至少持续运行30分钟。
- 检查无DMA错误、无接收块序号断裂、无PSRAM环形缓冲溢出、无SD写入错误。
- 对导出的ADC帧执行现有同步/格式检查，确认新增LCD_CAM/GDMA没有破坏30 MHz
  数据通路。

## 13. 完成标准

只有同时满足以下条件才能宣布完成：

- ESP-IDF v6.0.2完整构建通过。
- 自动协议和波形测试通过。
- 四个新增引脚无现有资源冲突。
- 示波器确认连续6.6 MHz、互补相位和40/61周期时序。
- GPIO6与GPIO7职责完全独立。
- 10个启动帧与本文黄金值完全一致。
- ADC采集和SD写入满负载回归无新增错误。


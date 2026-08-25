# ESP32-S3 30 MHz ADC Stream Logger

> 文档整合时间：2026-08-26
>
> 当前有效版本：ESP-IDF v6.0.2，ESP32-S3-WROOM-2-N32R16V
>
> 工程目录：`E:\ESP_IDF_File\ESP32S3_30M_ADC_STREAM_TO_SD`

本文档是本工程唯一的 README，合并了原根目录 README、`host_tests/README.md`、
全部历史 `.TXT/.txt` 说明文件，并按时间区分“当前有效设计”和“历史版本”。
历史记录用于追溯，不能代替当前配置。

## 1. 当前有效配置（2026-08-26）

### 1.1 默认行为与两个独立开关

- GPIO7：ADC/SD 记录开关，内部下拉；高、低电平均连续稳定 200 ms 才切换状态。
- GPIO5：刺激开关，内部下拉；电平连续稳定 100 µs 后才切换状态。
- GPIO7、GPIO5 均低或悬空：不写 ADC 数据；刺激端持续输出 FrameStop/IDLE。
- 仅 GPIO7 高：采集 ADC 并写入 SD，不启动刺激序列。
- 仅 GPIO5 高：发送刺激启动序列并维持刺激状态，不写入 SD。
- 两者均高：ADC/SD 记录与刺激同时运行。
- 记录过程中拔掉 GPIO7：排空缓存并关闭当前 SD 段；GPIO5 若仍高，刺激继续。
- 刺激过程中 GPIO5 变低：完成当前 101-clock slot 后返回 FrameStop 循环。

### 1.2 引脚表

| 功能 | GPIO | 方向 | 当前配置 |
| --- | ---: | --- | --- |
| ADC 时钟 | 20 | 输入 | 外部 30 MHz，上升沿采样 |
| ADC 串行数据 | 16 | 输入 | MSB first，无外部 CS |
| ADC/SD 记录开关 | 7 | 输入 | 内部下拉，200 ms 连续稳定消抖 |
| 刺激开关 | 5 | 输入 | 内部下拉，100 µs 连续稳定消抖 |
| 刺激 `mclkST` | 19 | 输出 | 连续 6.600 MHz |
| 刺激 `SCLK` | 8 | 输出 | 连续 6.600 MHz，与 GPIO19 反相 |
| 刺激 `MOSI` | 17 | 输出 | MSB first，在 SCLK 下降沿附近更新 |
| 刺激 `CSb` | 15 | 输出 | 低有效，40 clocks 低 + 61 clocks 高 |
| SDMMC CLK | 41 | 输出 | 20 MHz |
| SDMMC CMD | 42 | 双向 | 原生 SDMMC |
| SDMMC D0/D1/D2/D3 | 40/39/1/2 | 双向 | 4-bit 总线 |

### 1.3 ADC 帧格式

每帧固定为 260 字节（2080 bit）：

```text
字节 0..3     FF FF 00 00
字节 4..5     CH0（高字节在前）
字节 6..7     00 00 padding
字节 8..9     CH1
字节 10..11   00 00 padding
...
字节 256..257 CH63
字节 258..259 00 00 padding
下一字节立即开始下一帧 FF FF 00 00
```

计算为 `4-byte header + 64 × (2-byte sample + 2-byte padding) = 260 bytes`。
协议没有帧计数器、CRC、校验和或额外帧尾，ESP32 不改变 ADC 发送格式。

### 1.4 FreeRTOS、DMA 与 PSRAM 流水线

```text
SPI2/GDMA（CPU0/P20）
    -> 4 × 32 KiB 内部 DMA block
    -> 2 MiB PSRAM raw ring
    -> 帧解析（CPU0/P10）
    -> 12 MiB PSRAM valid ring
    -> 64 KiB 内部写缓存
    -> SDMMC（CPU1/P6）
```

- 输入速率：30 Mbit/s，即约 3.75 MB/s。
- raw ring 理论承接时间约 0.56 秒。
- valid ring 理论承接时间约 3.36 秒。
- DMA 任务只搬运并快速归还硬件块；解析和 SD 写入不占用 DMA 块。
- 锁定后的解析采用批量高速路径；0-bit 对齐时直接校验完整帧，避免逐字节函数调用。
- DMA 序号断裂、描述符错误、PSRAM 环溢出和 SD 写入错误均为致命错误。

### 1.5 当前帧同步与恢复策略

状态机为：

```text
ACQUIRE -> COHORT -> LOCKED -> HOLDOVER
               ^                 |
               +-----------------+
```

- 初始锁定和真正失步后的全局重锁使用同一规则。
- 搜索全部 0～7 bit 偏移。
- 同一 bit 相位连续出现 8 个、间隔 2080 bit 的 `FFFF0000` 后形成候选。
- 第一个候选确认后，再观察一个 260-byte 周期，统计同批其他候选。
- 不再使用曾导致处理不及时的 8192 帧长验证。
- 8 个确认帧和 1 个候选观察帧不写入 SD，从下一帧开始写入；30 Mbit/s 下约损失 0.624 ms。
- 只有一个候选时正常记录。
- 存在多个候选时选择最先确认的候选继续记录，并把整段标记为
  `CLOSED_UNCERTAIN_SYNC`，提示通道可能循环错位。
- 已锁定后发现单帧帧头或 padding 错误，丢弃坏帧并保留原 2080-bit 相位。
- 在原相位最多观察 8 帧；连续 4 帧正确后，从下一帧恢复写入。
- 原相位恢复失败则立即全局重搜，不再固定盲丢弃 10 秒。
- 搜索超过 10 秒只输出 `SYNC UNRESOLVED`，DMA 与搜索继续运行。

段结束状态：

- `CLEAN`：没有结构错误或失步。
- `CLOSED_WITH_GAPS`：发生错误，但停止前均已恢复。
- `CLOSED_UNCERTAIN_SYNC`：出现多个候选，选择了最先确认的候选。
- `FAILED_UNRESOLVED_SYNC`：停止时仍未恢复同步。
- `FAILED_PIPELINE`：DMA、缓冲区或 SD 写入错误。
- PC 工具仍兼容旧版 `FAILED_AMBIGUOUS_SYNC`。

协议固有限制：若某个 ADC 通道长期为 `FFFF`，其后的固定 `0000` padding 会形成
周期性伪帧头。仅靠时钟和数据两根线，软件无法绝对证明哪个候选是真帧头，也无法
发现发送端恰好漏掉一整个 260-byte 帧。彻底解决需要：

1. 增加独立 FRAME/POR 帧同步线；或
2. 约定一个 ADC 通道保持已知且不会等于 `FFFF` 的参考值，例如固定接地，具体编码需按 ADC 手册确认。

### 1.6 裸 SD 布局

本工程不挂载 FAT，不需要预建 `RAW_DATA.BIN` 或 `REC_INFO.TXT`。

| LBA 范围 | 用途 |
| --- | --- |
| 0、1 | 双超级块 |
| 2..2047 | 分段目录和同步诊断元数据 |
| 2048 起 | 原始数据区，容量为 SD 卡剩余的全部扇区 |

- 每次重新上电仍从 LBA2048 开始覆盖上一轮数据，不做跨上电追加。
- 采集过程中只顺序写数据区，不周期性跳回 LBA0/LBA1 改写进度；GPIO7 正常停止并关闭段时才记录精确结束 LBA。
- 下次启动先读取并校验两份超级块，选择代数最新的一份，并以记录的结束 LBA 作为上一轮实际使用范围。
- 启动擦除范围为“元数据区 + 上一轮已用数据 + 4 MiB余量”，末端按 SD 卡报告的 Allocation Unit 向上取整；卡未报告 AU 时按 4 MiB取整。
- 例如上一轮约写入 5 GiB，下一轮只擦除略大于 5 GiB 的范围，不再固定擦除 10 GiB。
- 如果上一轮异常断电，超级块仍会标记为运行中，程序会提示记录的结束 LBA 可能不准确；这不会改变新一轮从 LBA2048 覆盖写入的规则。
- 首次使用或两份旧元数据都无效时，程序无法推断上一轮范围，会打印警告并跳过定向预擦除；擦除命令失败则本次采集不启动。
- 同次上电中，GPIO7 每次稳定变高创建新段，稳定变低时排空缓存并关闭该段。
- FAILED 段保留已经确认落盘的完整帧字节数和帧数。
- 同步事件、候选数量、恢复次数、丢弃量和 DMA 序号写在元数据区，不插入 ADC 的 260-byte 帧。

> **警告：** 固件会覆盖 SD 卡的分区表、文件系统和原始数据。Windows 会把卡显示为
> 未格式化；不要点击格式化，也不要使用存有重要数据的卡。

### 1.7 FPGA 刺激端口（方案 A）

刺激波形由 LCD_CAM 8-bit 并行发送端和循环 AHB GDMA 产生：

- DMA byte 仅使用 bit0（MOSI）和 bit1（CSb），bit7:2 恒为 0。
- LCD 时钟源为 PLL160，核心分频为 `12 + 4/33`，PCLK 再分频 2，输出 6.600 MHz。
- GPIO19 与 GPIO8 输出两路反相 6.600 MHz 时钟。
- 每个 slot 为 CSb 低 40 clocks、高 61 clocks，共 101 clocks。
- GPIO5 低时循环输出 STOP/IDLE；稳定变高后只发送一次 10 个 5-byte 配置帧，
  随后进入 `ENABLED_IDLE`，高电平期间不重复发送启动序列。
- LCD_CAM 循环链启动前增加 2 个 `CSb=1, MOSI=1` 空闲 PCLK，以规避 ESP32-S3 LCD-239 勘误。
- 常态循环不产生逐帧中断，避免持续占用 ADC 解析所需 CPU。
- 刺激模块发生故障时立即停止刺激 DMA/时钟并拉高 CSb/MOSI，只记录错误；
  ADC/SPI2、PSRAM、同步器和 SDMMC 继续运行。

GPIO8、GPIO19 是连续高速时钟，应尽量短线、与 FPGA 共地，并用示波器实测后选择
GPIO 驱动强度，不要默认使用最大驱动能力。

## 2. 构建、测试与使用

### 2.1 构建与烧录

进入 ESP-IDF v6.0.2 命令行环境：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

把 `COMx` 替换为开发板实际串口，按 `Ctrl+]` 退出监视器。

### 2.2 主机自动测试

从工程根目录执行：

```powershell
python -m unittest discover -s host_tests -p "test_*.py" -v
```

测试覆盖 ADC/SD 架构边界、同步规则、刺激协议字段、10 个黄金帧、40/61 clock slot、
MSB first、静态缓冲区、FrameStop 循环、GPIO5 去抖、引脚资源冲突和刺激故障监测。
ESP-IDF 完整构建仍是 C 源码的权威编译检查；刺激时序必须在目标板上实测。

### 2.3 导出 SD 数据

优先把采集卡制作成只读镜像；直接读物理卡时必须使用管理员权限并反复核对磁盘编号。

```powershell
python tools/read_raw_sd_segments.py --image capture.img
python tools/read_raw_sd_segments.py --image capture.img --segment 1 --output-dir exported
```

读取工具兼容元数据 v1/v2。`CLOSED_UNCERTAIN_SYNC` 可以导出，但分析前必须核对通道顺序。
用 `python tools/read_raw_sd_segments.py -h` 查看当前全部参数。

### 2.4 板上验收

ADC/SD：

1. GPIO7 稳定拉高后应出现 `SYNC LOCKED` 和 `RECORDING STARTED`。
2. 连续采集至少 60 秒，确认 `dma_overrun=0`、`raw_overflow=0`、`valid_overflow=0`。
3. GPIO7 拉低后等待段关闭日志，再断电或拔卡。
4. 导出数据，检查有效字节为 260 的整数倍，并核对通道顺序。

刺激：

1. 示波器确认 GPIO8 为 6.600 MHz，GPIO19 与其反相。
2. 确认 CSb 每个 slot 低 40 clocks、高 61 clocks，MOSI 为 MSB first。
3. 测量 GPIO5 稳定沿到 slot 边界切换的延迟。
4. ADC/SD 与刺激并发运行至少 30 分钟，确认 DMA、PSRAM 和 SD 无新增错误。

## 3. 按时间整理的修改历史

### 2026-08-09：最初的 SPI/FAT 文件写入设计（已废弃）

最初方案使用 FPGA/SPI Slave、DMA 双缓冲、8 MiB PSRAM ring、64 KiB 聚合缓存，
写入电脑预建的 `/sdcard/RAW_DATA.BIN`。SDMMC 已确定使用 GPIO41/42/40/39/1/2，
并把录制开关规划为 GPIO18；停止时要求排空缓存、处理短写/EINTR、调用 fsync/close，
速率只按实际写成功字节统计。

该阶段仍假定 SPI CS=GPIO10、DATA=GPIO11、CLK=GPIO12、mode 1，并使用 FAT 文件系统；
这些内容均已被后续“GPIO20/GPIO16 两线输入 + 裸 SD”实现取代。

### 2026-08-13：方案 A 三阶段流水线（部分沿用、协议假设已废弃）

为解决逐 bit 搜索占用 CPU、DMA block 归还不及时、`dma_overrun` 和看门狗问题，
采集被拆分为 DMA 搬运、帧解析、SD 写入三个 FreeRTOS 阶段；应用优化改为 `-O2`，
CPU 维持 240 MHz，Octal PSRAM 维持 80 MHz。

当时仍错误假定帧为 262 字节、ESP32 下降沿采样、末尾带 16-bit 大端帧计数器，
并使用 4 MiB raw + 8 MiB valid ring、FAT/RAW_DATA.BIN。该协议假设于次日被真实 FPGA
bitstream 推翻。

历史验证：37 项测试通过，固件 321,264 bytes，SHA-256：
`784CDBADD10190229B80C897FBB1497EDCE2E1E9EE5A04BD3B6452FF30ECF2DF`。

### 2026-08-14：确认真实 260-byte/MSB/上升沿协议

依据真实 TOP.v、OLVDS.v 和采集格式，确定 FPGA 在 30 MHz 下降沿更新数据，ESP32 应在
上升沿采样，即 SPI mode 0、MSB first。帧长由 262 改为 260 字节，删除不存在的计数器、
CRC 和帧尾；同步规则改为按 260-byte 间隔查找帧头并校验 64 个 `0000` padding。

历史验证：34 项测试通过，固件 321,360 bytes，SHA-256：
`1AD9BEC16C5F15E0A86A5C676ADAFE9D581C6F0D387C8CB09AB604A27F94D460`。

### 2026-08-14：锁定态解析器批量高速路径

删除锁定态逐字节 `process_locked_byte()` 调用，增加统一帧校验/输出入口：

- 0-bit 对齐时，跨 DMA block 的半帧用 `memcpy` 补齐，连续完整帧直接在输入块校验。
- 非 0-bit 对齐时，在紧凑循环中重组字节，跨块只保存前一原始字节和未完成帧。
- `raw_bits`、`raw_byte_position` 改为按已消费块批量更新。

历史验证：35 项测试通过，固件 321,840 bytes，SHA-256：
`D9C2354B3B0BF75F082BCC5C311BF7874F91EDAC8B5431690BF667AFD50EF510`。

### 2026-08-19：从 FAT 文件改为裸 SD 分段记录

取消 FAT 挂载、预建 `RAW_DATA.BIN` 和 `REC_INFO.TXT`，改为 SDMMC 4-bit 裸扇区顺序写入。
建立 LBA0/1 双超级块、LBA2..2047 分段目录和 LBA2048 起的 1 GiB 数据区；同次上电支持
多段记录，FAILED 段保留已经确认落盘的数据。

同时将 PSRAM 从 4 MiB raw + 8 MiB valid 调整为 2 MiB raw + 12 MiB valid，以增加
SD 卡偶发慢写时的承接时间。历史验证为 51 项测试通过、ESP-IDF 构建成功。

### 2026-08-20：建立 Git 仓库

工程开始使用 Git 管理。此前历史文档中的“本目录不是 Git 仓库”仅描述当时状态，
现已失效。

### 2026-08-22：采集工程与测速工程分离，开关改为 GPIO7

从采集工程移除备用 1 GiB SD 写入 benchmark 入口；测速代码单独保存在
`ESP32S3_Raw_SD_Write_Benchmark`。ADC/SD 记录开关从 GPIO18 改为 GPIO7，并使用
高、低电平均连续稳定 200 ms 的消抖。历史文件中的 GPIO18 均不再代表当前接线。

### 2026-08-23：加入 FPGA 刺激方案 A

加入 `stim_protocol`、`stim_waveform` 和 `stim_controller`：用 LCD_CAM + 循环 AHB GDMA
产生两路反相 6.600 MHz 时钟、MOSI 和 CSb，发送 10 个刺激配置帧，并增加 LCD-239
启动勘误规避、运行故障监测和安全拉高 CSb/MOSI。刺激故障不得中断 ADC/SD。

### 2026-08-24：PC 波形绘图工具

增加并调整通道 0 标准波形与 SD 波形绘图功能，放大坐标轴和标题。该工具属于离线分析，
不改变 ESP32 固件的采集、同步或 SD 数据格式。

### 2026-08-25：刺激开关 GPIO6 改为 GPIO5

刺激使能输入由 GPIO6 改为 GPIO5，内部下拉和 100 µs 消抖逻辑保持不变；GPIO7 仍只控制
ADC/SD 记录。修改后 21 项主机测试通过，ESP-IDF 完整构建成功，固件大小 297,760 bytes。

### 2026-08-26：文档统一整理

把根 README、主机测试 README、旧设计说明、协议修改记录、解析器优化记录、裸 SD 修改记录
和备注统一整合为本文档。旧配置均保留在带日期的历史章节，并明确标记是否已经废弃。

### 2026-08-26：按上一轮实际范围预擦除

正式采集器移除历史遗留的固定 1 GiB 数据区上限，实际容量改为 LBA2048 到 SD 卡末尾；
采集时长只由 GPIO7 停止动作或 SD 卡空间耗尽决定。为保持数据区连续顺序写入，采集中
不再周期性改写 LBA0/LBA1；正常停止时记录精确结束 LBA。下次上电按该位置增加 4 MiB
余量，并按 Allocation Unit 向上取整后执行擦除；异常断电时会告警结束位置可能不准确。
无论擦除范围多大，新一轮都仍从 LBA2048 开始覆盖，不跨上电追加。

## 4. 仓库范围

发布分支只应包含固件源码、构建配置、测试/读取工具和本 README；不包含 `build` 目录和
采集数据。当前采集工程与独立测速工程互不依赖。

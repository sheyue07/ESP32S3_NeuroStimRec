# ESP32S3 ADC eMMC Controller

本工程用于在 ESP32-S3 上连续接收外部 30 MHz 串行 ADC 数据，完成帧同步后写入裸 eMMC，并通过 UART0 供上位机查询、扫描和导出。

当前版本使用统一的 eMMC 管理任务串行执行所有存储操作。GPIO7 采集控制任务和 UART 上位机任务都不能直接访问 eMMC，从而避免采集、元数据收尾和数据导出互相冲突。

## 已验证状态

- ESP-IDF：6.0.2
- 芯片目标：ESP32-S3
- ADC 输入：30 MHz，MSB first，上升沿采样
- eMMC：SDMMC 4-bit，配置上限 20 MHz
- 控制串口：UART0，921600 baud，8N1
- 实测连续有效写入：约 3.57 MiB/s
- 实测 UART 导出：约 89.6 KiB/s，接近 921600 baud 的 8N1 理论上限
- 已验证 113.81 MiB 分段正常停止、导出并通过总 CRC32
- 当前固件镜像约 320 KiB，1 MiB 应用分区剩余约 69%

> 注意：工程使用 eMMC 裸扇区，不使用 FAT 文件系统。开始一个新 run 会从固定数据区起点覆盖此前采集内容。

## 系统结构

```text
GPIO7 写入开关 ──开始/停止请求──┐
                                │
ADC CLK/DATA → SPI2/GDMA        ▼
              → 原始环形缓冲 → 帧同步 → 有效帧环形缓冲
                                        │
                                        ▼
                                 eMMC 统一管理任务
                                 IDLE / WRITING
                                 FINALIZING / READING / ERROR
                                        ▲
                                        │
上位机 ←──── UART0 / EMB1 ──── UART 协议任务
```

任务和缓冲区配置：

| 环节 | CPU / 优先级 | 缓冲 |
|---|---:|---:|
| SPI2/GDMA 搬运 | CPU0 / 20 | 4 × 32 KiB 内部 DMA 块 |
| 原始流解析 | CPU0 / 10 | 2 MiB PSRAM byte ring |
| 有效帧写盘 | CPU1 / 6 | 12 MiB PSRAM no-split ring |
| eMMC 写缓存 | CPU1 / 6 | 64 KiB 内部 DMA 缓冲 |

工程至少需要能够容纳约 14 MiB 环形缓冲及其他工作区的 PSRAM；当前配置使用 80 MHz Octal PSRAM。

## 引脚定义

| 功能 | GPIO | 方向 | 说明 |
|---|---:|---|---|
| ADC CLK | 20 | 输入 | 30 MHz 外部时钟 |
| ADC DATA | 16 | 输入 | 串行 ADC 数据，MSB first |
| 采集开关 | 7 | 输入 | 高电平开始，低电平停止；内部下拉 |
| UART0 TX | 43 | 输出 | TXD0，封装 37 脚 |
| UART0 RX | 44 | 输入 | RXD0，封装 36 脚 |
| eMMC CLK | 10 | 输出 | SDMMC clock |
| eMMC CMD | 9 | 双向 | SDMMC command |
| eMMC D0 | 13 | 双向 | 4-bit data |
| eMMC D1 | 14 | 双向 | 4-bit data |
| eMMC D2 | 12 | 双向 | 4-bit data |
| eMMC D3 | 11 | 双向 | 4-bit data |
| 刺激使能 | 5 | 输入 | 独立于 GPIO7，100 us 消抖 |
| 刺激 MCLK | 19 | 输出 | LCD_CAM/GDMA 生成 |
| 刺激 SCLK | 8 | 输出 | 约 6.6 MHz |
| 刺激 MOSI | 17 | 输出 | 串行命令数据 |
| 刺激 CSb | 15 | 输出 | 低有效 |

UART、eMMC 和 GPIO7 引脚可在 `idf.py menuconfig` 的 `ADC eMMC controller configuration` 中修改。ADC 和刺激接口引脚目前由头文件固定。

## ADC 帧格式

每个有效帧固定为 260 字节：

```text
偏移 0..3      FF FF 00 00
偏移 4..259    64 × (16-bit ADC sample + 00 00 padding)
```

- 每个 ADC sample 按高字节在前接收。
- 帧内没有计数器。
- 同步器会搜索全部 8 种位移和 2080 种帧相位。
- 连续确认 8 个帧头后锁定。
- 锁定后同时校验帧头和每通道的 `00 00` 填充。
- 短暂错误进入 HOLDOVER；无法恢复时重新执行全局同步搜索。
- 只有结构校验通过的完整帧会进入 eMMC 数据区。

单个 run 的最大完整帧数为 4,129,776 帧，有效字节数为 1,073,741,760；补齐最后一个扇区后物理占用正好为 1 GiB。

## GPIO7 与状态机

GPIO7 每 10 ms 采样一次，连续两次一致才确认变化。

```text
IDLE --GPIO7 高--> WRITING --GPIO7 低/写满/错误--> FINALIZING
  ^                                                        |
  |------------------- 成功收尾 ---------------------------|

IDLE --UART 读取--> READING --> IDLE
任意严重采集或存储错误 -----------------------> ERROR
```

互锁规则：

| 状态 | 是否允许采集 | 是否允许 eMMC 读取 |
|---|---|---|
| IDLE | GPIO7 高可启动 | 仅 GPIO7 低时允许 |
| WRITING | 正在采集 | 不允许 |
| FINALIZING | 停止 DMA、排空缓冲、提交元数据 | 不允许 |
| READING | 不允许启动采集 | 正在执行读取或导出 |
| ERROR | 不允许 | 先执行 `REINIT` |

操作时应让 GPIO7 保持低电平，直到状态从 `FINALIZING` 回到 `IDLE`，然后再进行下一次低到高触发。过快地低—高切换可能在收尾期间被拒绝，且不会自动重试。

## eMMC 裸盘布局

扇区大小固定为 512 字节，元数据格式版本为 2。

| LBA | 用途 | 容量 |
|---:|---|---:|
| 0 | 超级块 A | 1 sector |
| 1 | 超级块 B | 1 sector |
| 2..255 | 分段目录 | 254 entries |
| 256..2047 | 同步事件区 | 12,544 events |
| 2048..2099199 | ADC 数据区 | 1 GiB |

每个分段记录以下信息：

- 分段状态、run ID、起始 LBA；
- 物理写入长度、有效数据长度、完整帧数；
- 开始/结束时间；
- 同步结果、重同步次数和丢弃字节；
- DMA 最后序号和序号间断；
- 事件区域位置和事件数量；
- 512 字节元数据校验值。

超级块保存两份，每写入约 64 MiB 数据更新一次运行元数据。断电后，上位机可以选择有效且 generation 更新的一份重新扫描。

首次采集前仍可扫描和导出上次留在 eMMC 中的数据；本次启动后的第一次 GPIO7 采集会建立新 run，并从 LBA 2048 开始覆盖旧数据。`REINIT` 本身不擦除数据，但 `REINIT` 后的下一次采集同样会开始新 run。

## UART 协议

默认端口为 UART0，921600 baud，8N1，无硬件流控。

支持的文本命令：

| 命令 | 用途 |
|---|---|
| `PING` | 握手 |
| `STATUS` | 查询内存状态和写入进度，不直接读 eMMC |
| `INFO` | 查询容量、eMMC 时钟和格式版本 |
| `LIST` | 扫描分段目录 |
| `DATA <index> [offset] [length]` | 导出指定分段的有效数据 |
| `READ <lba> <sector_count>` | 按 LBA 读取裸扇区 |
| `EVENTS <index>` | 导出分段同步事件扇区 |
| `REINIT` | 低电平且非忙状态下重新初始化 eMMC |
| `HELP` | 查看命令帮助 |

`DATA`、`READ` 和 `EVENTS` 使用 EMB1 二进制流：

```text
4 bytes   magic = "EMB1"
4 bytes   packet sequence
8 bytes   stream offset
4 bytes   payload bytes
4 bytes   payload CRC32
N bytes   payload，最大 4096 字节
```

每包携带 CRC32，流结束时再发送整个数据流的总 CRC32。这里的 CRC 用于检测 UART 传输错误，不参与 ADC 写入速度统计。写入期间不计算全量 eMMC 数据 CRC，因此目录中的 `data_crc` 当前为 0。

UART0 同时是默认日志控制台。协议初始化成功后，固件会关闭 ESP-IDF 日志输出，确保日志不会混入 `STATUS` 文本或 EMB1 二进制流。调试底层错误时应临时使用另一串口，或扩展现有协议返回诊断字段。

## 编译与烧录

请使用已经加载 ESP-IDF 6.0.2 环境的 PowerShell 或 ESP-IDF VS Code 扩展：

```powershell
cd E:\ESP_IDF_File\ESP32S3_ADC_eMMC_Ctrl
idf.py build
idf.py -p COM4 flash
```

如需完全重建：

```powershell
idf.py fullclean
idf.py build
```

主要产物：

```text
build\ESP32S3_0202.bin
build\bootloader\bootloader.bin
build\partition_table\partition-table.bin
```

根 CMake 工程名仍为历史名称 `ESP32S3_0202`，所以固件文件使用该名称；这不影响运行。

## 推荐操作流程

1. 上电前保持 GPIO7 低电平。
2. 启动上位机并连接 UART0，执行握手和 `STATUS`。
3. 如需保留旧数据，先执行 `LIST` 和导出。
4. GPIO7 拉高，开始正式 ADC 采集。
5. 写入期间只轮询 `STATUS`，不要发起 eMMC 读取。
6. GPIO7 拉低，并保持低电平等待 `FINALIZING` 完成。
7. 状态回到 `IDLE` 后扫描分段并导出数据。
8. 等待上位机报告总 CRC32 校验通过。

上位机若仍显示“模拟数据写入测速”或“抽样校验 0 次”，这是沿用旧界面的文字。当前固件写入的是真实 ADC 帧，正式采集路径不执行模拟测速版本的抽样回读。

## 分段结果码

| `capture_outcome` | 含义 |
|---:|---|
| 1 | `CLEAN`，同步稳定、正常关闭 |
| 2 | `CLOSED_WITH_GAPS`，发生过失锁但已恢复 |
| 3 | `FAILED_UNRESOLVED_SYNC`，结束时未建立或未恢复同步 |
| 4 | `FAILED_AMBIGUOUS_SYNC`，保留的旧格式状态 |
| 5 | `FAILED_PIPELINE`，DMA、环形缓冲或 eMMC 管线错误 |
| 6 | `CLOSED_UNCERTAIN_SYNC`，锁定时存在多个候选相位 |

即使分段为 `FAILED`，已经提交的完整有效帧通常仍可在重启或 `REINIT` 后导出。定位同步问题时应导出 `EVENTS`，而不是只检查有效数据 BIN，因为无效原始区间不会被写入有效数据文件。

## 代码模块

| 文件 | 职责 |
|---|---|
| `main/main.c` | 初始化顺序和程序入口 |
| `main/write_switch.c` | GPIO7 采集开关和消抖 |
| `main/emmc_storage_manager.c` | ADC 管线、状态机、唯一 eMMC 所有者 |
| `main/continuous_rx.c` | SPI2/GDMA 连续串行接收 |
| `main/frame_sync.c` | 260 字节帧同步、校验和重同步 |
| `main/raw_sd_segment_recorder.c` | 64 KiB 写缓存和裸盘分段记录 |
| `main/raw_sd_segment_format.c` | 超级块、目录和事件扇区格式 |
| `main/uart_bridge.c` | UART 文本命令、EMB1 导出和 CRC32 |
| `main/stim_*.c` | GPIO5 控制的刺激命令波形 |
| `main/Kconfig.projbuild` | UART、GPIO7 和 eMMC 可配置引脚 |

## 审查结论与已知边界

2026-08-28 对当前代码执行了核心并发路径审查和 ESP-IDF 6.0.2 全量编译。未发现会阻断已验证“采集—GPIO 停止—扫描—CRC 导出”主流程的确定性 bug，全部工程源文件无编译警告。

仍需注意以下边界：

- 后续分段的 `STATUS target` 仍固定显示 1 GiB，而不是当前 run 的剩余容量；只影响进度显示，容量检查仍按物理剩余空间执行。
- GPIO7 拉低时若同步器恰好处于 HOLDOVER 或全局搜索，当前收尾规则可能把分段记为 `FAILED_UNRESOLVED_SYNC`；已写入的有效帧仍保持完整。
- 多种管线错误最终会合并为结果码 5，协议目前没有持久化具体失败阶段；应结合 `STATUS`、目录和事件数据定位。
- 输入信号错误率过高会让同步器长期运行在较慢的全局搜索路径，2 MiB 原始环形缓冲可能溢出并触发结果码 5。
- `continuous_rx.c` 和 `stim_waveform.c` 使用 ESP-IDF 私有 HAL/GDMA 接口，升级 ESP-IDF 后必须重新全量编译并做硬件回归测试。
- 当前工程没有独立的自动化单元测试目录；现阶段验证依据为全量编译、元数据结构静态断言及真实硬件采集/导出测试。

# NeuroStimRec 神经信号采集刺激系统

本工程用于在 ESP32-S3 上连续接收外部 30 MHz 串行 ADC 数据，完成帧同步后全量写入裸 eMMC，并通过 BLE 向 Android APP 发送最多 8 个所选通道的降采样实时预览。采集和刺激均由手机蓝牙控制；原 UART0 查询、扫描和导出功能继续保留。

当前版本使用统一的 eMMC 管理任务串行执行所有存储操作。蓝牙控制任务和 UART 上位机任务都不能直接访问 eMMC，从而避免采集、元数据收尾和数据导出互相冲突。

## 已验证状态

- ESP-IDF：6.0.2
- 芯片目标：ESP32-S3
- ADC 输入：30 MHz，MSB first，上升沿采样
- eMMC：SDMMC 4-bit，配置上限 20 MHz
- 控制串口：UART0，921600 baud，8N1
- 蓝牙：NimBLE GATT，设备名 `NeuroStimRec`
- 手机预览：最多 8 通道、1–200 Hz；预览拥塞只丢预览包，不阻塞 ADC/eMMC
- ADC 满速输入约 3.58 MiB/s；当前将 eMMC 限制为 20 MHz，优先提高板级总线读写稳定性
- 实测 UART 导出：约 89.6 KiB/s，接近 921600 baud 的 8N1 理论上限
- 已验证 113.81 MiB 分段正常停止、导出并通过总 CRC32
- 当前含 NimBLE 的固件镜像约 607 KiB，1 MiB 应用分区剩余约 41%

> 注意：工程使用 eMMC 裸扇区，不使用 FAT 文件系统。开始一个新 run 会从固定数据区起点覆盖此前采集内容。

## 系统结构

```text
手机 BLE 控制 ─────采集请求──────┐
                                │
ADC CLK/DATA → SPI2/GDMA        ▼
              → 原始环形缓冲 → 帧同步 → 有效帧环形缓冲
                                  │             │
                                  │             └→ BLE 降采样预览
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
| 原始流解析 | CPU0 / 10 | 2 MiB PSRAM byte ring + 32.5 KiB 内部 RAM 批缓冲 |
| 有效帧写盘 | CPU1 / 8 | 12 MiB PSRAM no-split ring |
| eMMC 写缓存 | CPU1 / 8 | 64 KiB 内部 DMA 缓冲 |

工程至少需要能够容纳约 14 MiB 环形缓冲及其他工作区的 PSRAM；当前配置使用 80 MHz Octal PSRAM。高频访问的帧解析批缓冲保留在内部 RAM，避免采集解析与 PSRAM 环形缓冲争用外部存储总线。BLE 仅启用外设广播和 GATT Server 所需功能，未使用的 Central、Observer、安全配对和标准服务均关闭。

## 引脚定义

| 功能 | GPIO | 方向 | 说明 |
|---|---:|---|---|
| ADC CLK | 20 | 输入 | 30 MHz 外部时钟 |
| ADC DATA | 16 | 输入 | 串行 ADC 数据，MSB first |
| 原采集开关 | 7 | 输入 | 仅保留电平观测，不再控制采集 |
| UART0 TX | 43 | 输出 | TXD0，封装 37 脚 |
| UART0 RX | 44 | 输入 | RXD0，封装 36 脚 |
| eMMC CLK | 10 | 输出 | SDMMC clock |
| eMMC CMD | 9 | 双向 | SDMMC command |
| eMMC D0 | 13 | 双向 | 4-bit data |
| eMMC D1 | 14 | 双向 | 4-bit data |
| eMMC D2 | 12 | 双向 | 4-bit data |
| eMMC D3 | 11 | 双向 | 4-bit data |
| 原刺激使能 | 5 | 输入 | 不再配置或读取，不控制刺激 |
| 刺激 MCLK | 19 | 输出 | LCD_CAM/GDMA 生成 |
| 刺激 SCLK | 8 | 输出 | 约 6.6 MHz |
| 刺激 MOSI | 17 | 输出 | 串行命令数据 |
| 刺激 CSb | 15 | 输出 | 低有效 |

UART 和 eMMC 引脚可在 `idf.py menuconfig` 的 `ADC eMMC controller configuration` 中修改。ADC 和刺激接口引脚目前由头文件固定。

## ADC 帧格式

每个有效帧固定为 260 字节：

```text
偏移 0..3      FF FF 00 00
偏移 4..259    64 × (16-bit ADC sample + 00 00 padding)
```

- 每个 ADC sample 按高字节在前接收。
- ADC sample 是无符号 16 位数（`uint16_t`，范围 0..65535），不能按 `int16_t` 解释。
- 帧内没有计数器。
- 同步器会搜索全部 8 种位移和 2080 种帧相位。
- 连续确认 8 个帧头后锁定。
- 锁定后同时校验帧头和每通道的 `00 00` 填充。
- 短暂错误进入 HOLDOVER；无法恢复时重新执行全局同步搜索。
- 只有结构校验通过的完整帧会进入 eMMC 数据区。

单个 run 不再使用固定 1 GiB 上限。启动时按 eMMC 实际扇区数动态计算数据区：
`(物理扇区数 - 2048) × 512` 字节；每个分段末尾按 512 字节补齐，最后不足一帧
的空间不会写入。当前已验证设备的物理容量为 7,820,083,200 字节，可用数据区
为 7,819,034,624 字节。

## 手机蓝牙控制与状态机

GPIO7 和 GPIO5 不再参与采集或刺激状态切换。手机通过 BLE 发送带 `request_id` 的开始/停止命令，ESP32 执行后返回确认；蓝牙断开时释放手机采集请求并请求刺激安全停止。

```text
IDLE --BLE 开始--> WRITING --BLE 停止/写满/错误--> FINALIZING
  ^                                                        |
  |------------------- 成功收尾 ---------------------------|

IDLE --UART 读取--> READING --> IDLE
任意严重采集或存储错误 -----------------------> ERROR
```

互锁规则：

| 状态 | 是否允许采集 | 是否允许 eMMC 读取 |
|---|---|---|
| IDLE | BLE 命令可启动 | 允许 |
| WRITING | 正在采集 | 不允许 |
| FINALIZING | 停止 DMA、排空缓冲、提交元数据 | 不允许 |
| READING | 不允许启动采集 | 正在执行读取或导出 |
| ERROR | 不允许 | 先执行 `REINIT` |

停止采集后应等待 APP 显示状态从 `FINALIZING` 回到 `IDLE` 再启动下一次采集；收尾期间的开始命令会被拒绝。

## eMMC 裸盘布局

扇区大小固定为 512 字节，元数据格式版本为 2。

| LBA | 用途 | 容量 |
|---:|---|---:|
| 0 | 超级块 A | 1 sector |
| 1 | 超级块 B | 1 sector |
| 2..255 | 分段目录 | 254 entries |
| 256..2047 | 同步事件区 | 12,544 events |
| 2048..eMMC 最后一个 LBA | ADC 数据区 | 剩余全部物理容量 |

每个分段记录以下信息：

- 分段状态、run ID、起始 LBA；
- 物理写入长度、有效数据长度、完整帧数；
- 开始/结束时间；
- 同步结果、重同步次数和丢弃字节；
- DMA 最后序号和序号间断；
- 事件区域位置和事件数量；
- 512 字节元数据校验值。

超级块保存两份，每写入约 64 MiB 数据更新一次运行元数据。断电后，上位机可以选择有效且 generation 更新的一份重新扫描。

首次采集前仍可扫描和导出上次留在 eMMC 中的数据；本次启动后的第一次手机 BLE 采集会建立新 run，并从 LBA 2048 开始覆盖旧数据。`REINIT` 本身不擦除数据，但 `REINIT` 后的下一次采集同样会开始新 run。

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
| `REINIT` | 非忙状态下重新初始化 eMMC |
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

固件对导出过程中的瞬时 eMMC CRC/超时错误最多原位重读 3 次。若重读仍失败，
会在二进制流内返回明确的 `ERR READ` 并停止该次导出；上位机不会再把错误文本
误判成 EMB1 数据包并发送无效重传确认。

UART0 同时是默认日志控制台。协议初始化成功后，固件会关闭 ESP-IDF 日志输出，确保日志不会混入 `STATUS` 文本或 EMB1 二进制流。调试底层错误时应临时使用另一串口，或扩展现有协议返回诊断字段。

## BLE 与 Android APP

GATT 使用 NeuroStimRec 项目专属 UUID：

| 用途 | UUID |
|---|---|
| Service | `11E28B44-7380-4DB2-9B18-AFE970475001` |
| 手机写入 | `11E28B44-7380-4DB2-9B18-AFE970475002` |
| ESP32 通知 | `11E28B44-7380-4DB2-9B18-AFE970475003` |

主广播包包含专属 Service UUID，扫描响应包包含完整设备名
`NeuroStimRec`，从而保持传统 BLE 广播的 31 字节限制并避免与通用
Nordic UART Service 设备冲突。

应用层采用版本化二进制协议，含消息序号、分片序号和 CRC-16/CCITT-FALSE。支持状态查询、预览通道配置、采集启停、刺激参数/控制命令以及实时预览数据。预览通道统一使用 `CH0–CH63`：`CHn` 从有效帧偏移 `4 + n × 4` 提取。Android 工程位于 `E:\graduation_project\NeuroStimRec_Android`。

新版 `STATUS` 载荷为 56 字节，在原 40 字节状态末尾追加 `target_bytes`
和 `capacity_bytes` 两个 64 位 little-endian 字段。前者是当前分段开始时的剩余
可写容量，后者是 eMMC 物理容量；配套 APP 同时兼容旧版 40 字节状态。

实时预览直接从 GPIO20/16 接收后的已验证 260 字节帧中提取，不读取 eMMC。预览队列满时仅增加预览丢弃计数；完整帧仍按原路径写入 eMMC。

刺激配置采用 24 字节 BLE 载荷，字段名与 FPGA 保持一致：`Ch`、`ChipID`、`STclk_Sel`、`mode`、`Freq`、`PulseNum`、`PulseWA`、`PulseGap`、`PulseWC`、`PulseAMP`、`Stim`。ESP32 只在停止状态接受新配置，并使用双缓冲切换下一次启动序列；开始命令必须引用最近确认的配置版本。

## 编译与烧录

请使用已经加载 ESP-IDF 6.0.2 环境的 PowerShell 或 ESP-IDF VS Code 扩展：

```powershell
cd E:\ESP_IDF_File\ESP32S3_NeuroStimRec
idf.py build
idf.py -p COM4 flash
```

如需完全重建：

```powershell
idf.py fullclean
idf.py build
```

`sdkconfig.defaults` 保存了精简后的 BLE 和内部 RAM 配置。即使删除
`sdkconfig` 与 `build` 后重新配置，也会恢复本工程验证过的关键选项。

主要产物：

```text
build\NeuroStimRec.bin
build\bootloader\bootloader.bin
build\partition_table\partition-table.bin
```

根 CMake 工程名为 `NeuroStimRec`，与 BLE 广播名和总项目名一致。

## 推荐操作流程

1. 启动上位机并连接 UART0，执行握手和 `STATUS`。
2. 如需保留旧数据，先执行 `LIST` 和导出。
3. 在手机 APP 中连接 `NeuroStimRec`，选择预览通道并点击“开始采集”。
4. 写入期间只轮询 `STATUS`，不要发起 eMMC 读取。
5. 在手机 APP 中点击“停止采集”，等待 `FINALIZING` 完成。
6. 状态回到 `IDLE` 后扫描分段并导出数据。
7. 等待上位机报告总 CRC32 校验通过。

手机预览默认约 50 点/秒，只用于低频趋势观察；超过 25 Hz 的输入会混叠，不能据此判断高频波形是否失真。完整 14.4 kframe/s 左右的有效帧仍写入 eMMC，应以导出的 BIN/MAT 数据验证 1 kHz 等高频信号。

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
| `main/emmc_storage_manager.c` | ADC 管线、状态机、唯一 eMMC 所有者 |
| `main/continuous_rx.c` | SPI2/GDMA 连续串行接收 |
| `main/frame_sync.c` | 260 字节帧同步、校验和重同步 |
| `main/raw_sd_segment_recorder.c` | 64 KiB 写缓存和裸盘分段记录 |
| `main/raw_sd_segment_format.c` | 超级块、目录和事件扇区格式 |
| `main/uart_bridge.c` | UART 文本命令、2 KiB EMB1 分包、逐包 ACK/重传和 CRC32 |
| `main/stim_*.c` | 手机 BLE 控制的动态刺激命令波形 |
| `main/ble_protocol.c` | BLE 分片、CRC、命令解析和线格式 |
| `main/ble_service.c` | NimBLE GATT、状态通知与控制任务 |
| `main/adc_preview.c` | 从实时有效帧提取所选通道并限速预览 |
| `main/Kconfig.projbuild` | UART 和 eMMC 可配置引脚 |

## 审查结论与已知边界

2026-08-30 对采集热路径与 BLE 内存配置完成调整并通过 ESP-IDF 6.0.2 全量编译：静态 DIRAM 使用 96,595 字节，固件镜像约 613 KiB。该构建仍需在真实硬件上完成“BLE 扫描连接—30 秒采集—停止—扫描分段—导出”的回归验证。

仍需注意以下边界：

- `STATUS target` 是当前分段开始时的剩余可写容量；每关闭一个分段产生的扇区补齐会减少后续分段可用空间。
- 如果开始采集后 IO20 没有外部时钟，SPI/GDMA 会等待数据，设备保持 `WRITING`、帧数和写入字节为 0，BLE/UART 仍可响应；当前没有无时钟自动超时。此时手机停止命令可以结束管线，但空分段会记为 `FAILED_UNRESOLVED_SYNC`，状态进入 `ERROR`，再次采集前需执行 `REINIT`。
- 分段结果码 2 表示采集中确实发生过同步中断和恢复；应配合导出的 `EVENTS` 文件、FPGA 数据源和 IO16/IO20 逻辑分析结果定位，不能把它当作正常的干净采集。
- 多种管线错误最终会合并为结果码 5，协议目前没有持久化具体失败阶段；应结合 `STATUS`、目录和事件数据定位。
- 输入信号错误率过高会让同步器长期运行在较慢的全局搜索路径，2 MiB 原始环形缓冲可能溢出并触发结果码 5。
- `continuous_rx.c` 和 `stim_waveform.c` 使用 ESP-IDF 私有 HAL/GDMA 接口，升级 ESP-IDF 后必须重新全量编译并做硬件回归测试。
- 当前工程没有独立的自动化单元测试目录；现阶段验证依据为全量编译、元数据结构静态断言及真实硬件采集/导出测试。

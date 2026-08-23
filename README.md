# ESP32-S3 30 MHz ADC Stream Logger

## FPGA 刺激端口（方案 A）

本工程在不改变 ADC/SD 采集链路的前提下，增加独立的 FPGA 刺激端口：

- GPIO19：`mclkST`，连续 6.600 MHz；
- GPIO8：`SCLK`，连续 6.600 MHz，与 GPIO19 精确反相；
- GPIO17：`MOSI`，在 SCLK 下降沿附近更新；
- GPIO15：`CSb`，低有效，每帧低 40 个时钟、随后高 61 个时钟；
- GPIO6：刺激使能输入，内部下拉，连续稳定 100 µs 后才接受电平变化；
- GPIO7：仍只用于 ADC/SD 记录开关，连续稳定 200 ms，职责未改变。

波形由 LCD_CAM 8-bit 并行发送端和循环 AHB GDMA 产生。DMA byte 仅使用
bit0（MOSI）和 bit1（CSb）；bit7:2 恒为 0。LCD 时钟源固定为 PLL160，
分频为 `24 + 8/33`，PCLK 分频为 1，目标输出恰为 6.600 MHz。
LCD_CAM 仅在整条循环链启动时先输出 3 个 `CSb=1, MOSI=1` 的空闲时钟，
用于规避 ESP32-S3 LCD-239 勘误；随后每个 DMA slot 仍严格为 40+61 个时钟。

GPIO6 低电平时循环输出 STOP/IDLE slot；稳定变高后只发送一次规范规定的
10 个 5-byte 配置帧，再进入 ENABLED_IDLE；稳定变低时先完成当前 101-clock
slot，再返回 STOP_LOOP。常态 STOP/IDLE 循环不会产生逐帧中断，避免给现有
30 Mbit/s ADC 采集增加持续 CPU 负担。

刺激 GDMA 初始化或运行失败时，GPIO15/17 被切回 GPIO 输出并保持高电平，
错误只记录到日志；ADC/SPI2、PSRAM ring、帧同步和 SDMMC 任务仍继续运行。
固件分别统计 GDMA 描述符错误、两级 TX FIFO 欠载和 LCD_CAM 意外停止；运行故障
由 IRAM 中断路径立即停止刺激 DMA/时钟并拉高 CSb/MOSI，再由控制任务输出诊断日志。

GPIO8 与 GPIO19 是连续高速时钟。两线应尽量短、与 FPGA 共地，并在示波器实测后
选择合适的 GPIO 驱动强度；不要默认直接使用最大驱动强度。

### 刺激端口硬件验收

构建通过只代表软件可编译，烧录后还必须完成以下实测：

1. 示波器确认 GPIO8 为 6.600 MHz，GPIO19 与其反相；
2. 确认每个配置 slot 为 CSb 低 40 clocks、高 61 clocks，数据 MSB first；
3. 测量 GPIO6 稳定沿到 slot 边界切换的延迟；
4. 刺激输出和 30 MHz ADC/SD 同时运行至少 30 分钟，检查 DMA、PSRAM、SD
   无新增错误或溢出。

本项目使用 ESP32-S3 在外部 30 MHz 时钟下接收 ADC 串行数据，经 GDMA、PSRAM 环形缓冲区和帧同步器持续写入 SD 卡。

## 硬件与数据格式

- ADC 时钟：GPIO20，30 MHz，外部时钟，上升沿采样
- ADC 串行数据：GPIO16，MSB first
- 采集开关：GPIO7，高、低电平均需连续稳定 200 ms 才触发开始或停止
- SD 卡：SDMMC 4-bit，20 MHz
- ADC 帧：260 字节（2080 bit）
- 帧头：`FF FF 00 00`
- 帧体：64 组“16-bit ADC 样本 + `00 00` padding”

ADC 发送端的 260 字节格式没有被修改；所有同步诊断单独保存在 SD 元数据区域。

## 工程职责

本工程只负责 ADC 串行采集、帧同步和裸 SD 分段写入，不再包含 1 GiB SD 写入测速入口。测速固件已独立保存在 `ESP32S3_Raw_SD_Write_Benchmark` 工程，两个工程互不依赖。

## 同步恢复策略

状态机为 `ACQUIRE -> COHORT -> LOCKED -> HOLDOVER`，必要时从 `HOLDOVER` 回到全局搜索。

- 初始锁定与全局重锁使用相同规则：同一 bit 相位连续出现 8 个、间隔 2080 bit 的帧头。
- 第一个候选确认后，再观察一个 260 字节周期，统计同批通过 8 帧规则的其他候选。
- 不再执行原来的 8192 帧长验证；观察周期结束后立即选择最先确认的候选。
- 8 个确认帧和 1 个候选观察帧都不写入 SD，从其后的下一帧开始写入。在 30 Mbit/s 下启动损失约 0.624 ms。
- 若只有一个候选，正常记录；若有多个候选，仍选择第一个并继续记录，但整段永久标记为 `CLOSED_UNCERTAIN_SYNC`。
- 多候选可能由某些 ADC 通道持续输出 `FFFF`，并与其后的 `0000` padding 组成周期性伪帧头。此时输出的 260 字节帧结构仍正确，但通道顺序可能发生循环错位，必须结合标准数据核对。
- 已锁定后出现结构错误，坏帧被丢弃，并在原相位最多观察 8 帧。
- 原相位连续 4 帧正确后恢复写入；4 个证明帧也被丢弃。
- 原相位恢复失败后立即按上述快速 8 帧规则全局重搜，不再固定盲丢弃 10 秒。
- 搜索超过 10 秒只打印 `SYNC UNRESOLVED` 告警，DMA 和搜索仍继续运行。
- DMA 序号断裂、DMA/描述符错误、PSRAM 缓冲区溢出和 SD 写入错误仍是致命错误。

段结束状态为 `CLEAN`、`CLOSED_WITH_GAPS`、`CLOSED_UNCERTAIN_SYNC`、`FAILED_UNRESOLVED_SYNC` 或 `FAILED_PIPELINE`。读取工具仍兼容旧固件写出的 `FAILED_AMBIGUOUS_SYNC`。

软件只能报告多候选风险，无法仅凭当前 260 字节格式证明哪个候选是真帧头。彻底消除歧义的硬件方案是增加一根帧同步指示线，或让一个约定 ADC 通道保持已知且不会等于 `FFFF` 的参考值（例如固定接地，具体数值需按 ADC 编码确认）。

## 重要警告

固件使用专用裸 SD 模式，会覆盖 SD 卡的 LBA0/LBA1、元数据区和数据区。不要把需要保留数据的卡插入设备，也不要在 Windows 中格式化采集卡后继续期望读取原始段。

## 构建与烧录

需要 ESP-IDF v6.0.2。进入 ESP-IDF 命令行环境后执行：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

将 `COMx` 替换为开发板串口。按 `Ctrl+]` 退出监视器。

## 导出 SD 数据

先将采集卡制作成只读镜像，或直接以管理员权限读取物理磁盘。务必核对磁盘编号，避免选错设备。

```powershell
python tools/read_raw_sd_segments.py --image capture.img
python tools/read_raw_sd_segments.py --image capture.img --segment 1 --output-dir exported
```

读取工具兼容旧版 v1 和当前 v2 元数据；v2 会同时显示同步事件、候选数量、不确定标志、恢复次数、丢弃量、DMA 序号以及最终采集状态。`CLOSED_UNCERTAIN_SYNC` 数据可以导出，但分析前必须检查通道顺序。使用 `-h` 查看本机版本支持的全部参数。

## 测试

```powershell
python -m unittest discover -s host_tests -p "test_*.py" -v
```

刺激端口测试覆盖协议字段构造、10 个黄金帧、40/61 样本槽、MSB first、静态缓冲区、
FrameStop 循环、GPIO6 去抖、资源冲突、故障监测，以及 ADC/SD 关键实现文件未改动。

## 仓库范围

发布分支只包含固件源码、构建配置和本 README；不包含 `build` 目录及任何采集数据。

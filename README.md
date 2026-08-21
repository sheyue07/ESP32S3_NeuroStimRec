# ESP32-S3 30 MHz ADC Stream Logger

本项目使用 ESP32-S3 在外部 30 MHz 时钟下接收 ADC 串行数据，经 GDMA、PSRAM 环形缓冲区和帧同步器持续写入 SD 卡。

## 硬件与数据格式

- ADC 时钟：GPIO20，30 MHz，外部时钟，上升沿采样
- ADC 串行数据：GPIO16，MSB first
- 采集开关：GPIO18，高、低电平均需连续稳定 200 ms 才触发开始或停止
- SD 卡：SDMMC 4-bit，20 MHz
- ADC 帧：260 字节（2080 bit）
- 帧头：`FF FF 00 00`
- 帧体：64 组“16-bit ADC 样本 + `00 00` padding”

ADC 发送端的 260 字节格式没有被修改；所有同步诊断单独保存在 SD 元数据区域。

## GPIO7 与工程拆分设计（待实施）

本次改动将把正式 ADC 采集固件和 1 GiB SD 写入测速固件完全分开：

- 本工程及 GitHub 的 `main`、`ESP32_Code_0820` 分支只保留 ADC 采集入口；删除 `RAW_WRITE_BENCHMARK` 条件编译、`raw_write_benchmark.c/.h` 及对应 CMake 源文件。
- 两个采集分支的启停开关均由 GPIO18 改为 GPIO7；各分支现有的去抖和帧同步策略保持不变。
- `ESP32S3_Raw_SD_Write_Benchmark` 独立工程只保留 1 GiB 原始 SD 写入测速入口和必需的 SD 记录器依赖，不再编译 ADC 接收、帧同步和正式采集任务。
- 独立测速工程的开关也统一为 GPIO7，但与采集工程互不依赖。
- `ESP32_Code_0820` 的 README 将补充引脚、数据格式、启停方式、构建步骤、裸 SD 覆盖警告和该分支所使用的同步策略，不会误写成当前 `main` 的新算法。
- 验收要求为：采集工程不再包含 benchmark 符号或源文件；三个目标均不再引用 GPIO18；两个采集分支和独立 benchmark 均能通过 ESP-IDF 编译。

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
python -m unittest discover -s test -p "test_*.py"
```

测试覆盖按键去抖、帧格式、8 帧快速锁定、0～7 bit 偏移、单帧错误快速恢复、稀疏错误、bit 失步、多候选首选策略、段状态、元数据校验和读取工具兼容性。

## 仓库范围

发布分支只包含源码、构建配置、测试、读取工具和本 README；不包含 `build` 目录及任何采集数据。

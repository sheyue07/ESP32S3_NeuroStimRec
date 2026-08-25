# ESP32-S3 Stimulator FPGA Port（方案 A）实施计划

> **执行要求：** 严格以 `docs/superpowers/specs/2026-08-23-esp32s3-stimulator-fpga-port-design.md` 的方案 A 为准；每个任务遵循 RED → GREEN → REFACTOR，并在最终声明完成前执行完整构建与回归验证。

**目标：** 在现有 ESP32-S3 ADC→PSRAM→SDMMC 连续采集工程中增加独立的 FPGA 刺激端口，同时保持 ADC/SD 采集路径及数据格式不变。

**架构：** 新增三个边界清晰的模块：`stim_protocol` 只负责生成 5-byte 配置帧；`stim_waveform` 负责把帧编译为 LCD_CAM/GDMA 循环波形并驱动 GPIO19/8/17/15；`stim_controller` 负责 GPIO5 的 100 µs 连续稳定消抖和 STOP/START/IDLE 状态控制。刺激模块初始化失败时只报告错误并保持 CSb/MOSI 为高，不中止现有 ADC/SD 初始化。

**平台：** ESP-IDF v6.0.2、ESP32-S3、LCD_CAM i80 发送端、AHB GDMA、FreeRTOS、Unity/主机 Python 回归测试。

---

## 任务 1：建立可版本化的主机回归测试框架

**文件：**

- 新建：`host_tests/test_stim_protocol.py`
- 新建：`host_tests/test_stim_waveform.py`
- 新建：`host_tests/test_stim_integration.py`
- 新建：`host_tests/README.md`

1. 先写协议黄金帧、校验和、MSB-first、40/61 时隙、101/1010 buffer 长度测试。
2. 写集成约束测试：GPIO20/16/7、SDMMC pins、30 MHz SPI2 捕获定义不得变化；刺激端口必须是 GPIO19/8/17/15/6。
3. 运行 `python -m unittest discover -s host_tests -p "test_*.py" -v`，确认因实现文件缺失而失败。
4. 只提交测试骨架，不用测试放宽规范。

## 任务 2：实现刺激协议编码

**文件：**

- 新建：`main/stim_protocol.h`
- 新建：`main/stim_protocol.c`
- 修改：`main/CMakeLists.txt`
- 测试：`host_tests/test_stim_protocol.py`

1. 定义 5-byte 帧、10 条启动命令及模 256 校验和 API。
2. 使用固定宽度整数，禁止堆分配。
3. 生成并逐字节比对规范中的十条黄金帧。
4. 运行协议测试，确认全部通过。

## 任务 3：实现纯波形编译器

**文件：**

- 新建：`main/stim_waveform_builder.h`
- 新建：`main/stim_waveform_builder.c`
- 修改：`main/CMakeLists.txt`
- 测试：`host_tests/test_stim_waveform.py`

1. 定义 LCD_CAM DMA byte：bit0=MOSI、bit1=CSb、bit7:2=0。
2. 把每个 40-bit 帧按 MSB-first 编译为 40 个 CSb=0 样本，随后 61 个 CSb=1/MOSI=1 样本。
3. 编译 `stop_loop[101]`、`start_sequence[1010]`、`enabled_idle[101]`。
4. 对每个边界位、空闲位、总长度执行测试并通过。

## 任务 4：实现 LCD_CAM + 循环 GDMA 波形引擎

**文件：**

- 新建：`main/stim_waveform.h`
- 新建：`main/stim_waveform.c`
- 修改：`main/CMakeLists.txt`
- 测试：`host_tests/test_stim_integration.py`

1. 在内部 DMA RAM 静态预分配三个规范 buffer 和所有 `dma_descriptor_t`；运行期不生成波形、不分配 DMA buffer/descriptor。
2. LCD_CAM 时钟固定使用 PLL160。为规避 ESP32-S3 LCD-239，设置 `div_num=12, div_a=33, div_b=4` 得到 13.2 MHz LCD 核心时钟，再设 PCLK prescale=2，输出仍精确为 6.600 MHz；启动前发送两个高电平命令像素，使 DMA 数据前具有 4 个 LCD 核心周期。
3. GPIO8 连接 LCD PCLK，GPIO19 连接同一 PCLK 的反相信号；GPIO17/D0=MOSI，GPIO15/D1=CSb，不路由其他 LCD data pins。
4. 使用连续 AHB GDMA descriptor 链输出：STOP_LOOP 自循环；START_SEQUENCE 十个 101-byte descriptor；ENABLED_IDLE 自循环。
5. 状态切换只改 descriptor 边界链接，不停 LCD_CAM/GDMA；低电平请求不得截断正在发送的 40-bit 帧。
6. 注册 GDMA transaction/descriptor error 回调；故障时发布错误事件，GPIO15/17 回到高电平安全态，不影响 ADC 任务。
7. 用源代码契约测试检查时钟参数、GPIO matrix signal、descriptor 长度与禁止使用 SPI3/GPTimer/LEDC/bit-bang。

## 任务 5：实现 GPIO5 消抖与控制状态机

**文件：**

- 新建：`main/stim_controller.h`
- 新建：`main/stim_controller.c`
- 修改：`main/CMakeLists.txt`
- 测试：`host_tests/test_stim_integration.py`

1. GPIO5 配置为下拉输入、双边沿中断。
2. GPIO ISR 仅投递 edge event；控制任务收到事件后启动/重启 100 µs one-shot `esp_timer`。
3. timer callback 仅投递 expiry event；任务重新读取 GPIO5，只有电平在整个窗口未变化才提交状态切换。
4. 高电平只触发一次 10-frame START_SEQUENCE，然后进入 ENABLED_IDLE；低电平回 STOP_LOOP。
5. 使用静态 queue/task storage；回调不得构造 DMA 数据。

## 任务 6：集成但隔离现有 ADC/SD 采集

**文件：**

- 修改：`main/main.c`
- 修改：`main/CMakeLists.txt`
- 修改：`README.md`
- 测试：`host_tests/test_stim_integration.py`

1. 在 `app_main()` 中独立初始化刺激控制器；失败只打印错误并继续现有采集初始化。
2. 不修改 `continuous_rx.*`、`frame_sync.*`、`raw_sd_segment_*`、`record_metadata.*` 的实现。
3. README 补充刺激端口、协议、状态机、时钟、失败隔离及示波器验证方法。
4. 使用 `git diff b7f3247 --` 验证采集模块没有变动。

## 任务 7：构建、软件验证与硬件验收清单

**文件：**

- 更新：`README.md`（只记录真实完成的验证）

1. 运行 `python -m unittest discover -s host_tests -p "test_*.py" -v`。
2. 在已加载 ESP-IDF v6.0.2 环境下运行 `idf.py reconfigure build`。
3. 检查编译警告、IRAM/DRAM/flash 使用量和 map 中 LCD_CAM/GDMA 依赖。
4. 对照基线确认 ADC 30 MHz SPI2、GPIO7 开关、SDMMC 4-bit 20 MHz、260-byte 帧和 raw SD layout 未改变。
5. 输出硬件验收清单：示波器测 GPIO8=6.600 MHz、GPIO19 精确反相、40 low/61 high、GPIO5 切换延迟；ADC+刺激并发连续 30 min，确认 DMA/PSRAM/SD 无新增错误。
6. 无真实开发板/示波器结果时明确标记“硬件验证待执行”，不得把构建成功描述成硬件通过。

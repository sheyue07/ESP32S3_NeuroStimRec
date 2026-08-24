# Channel 0 Waveform Plot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Modify the existing MATLAB script so it opens two independent figures containing the complete channel 0 waveforms from the standard and SD MAT files.

**Architecture:** The script resolves both input files relative to its own location, loads only the required variables, validates their presence and shape, then extracts MATLAB row 1 and plots it in two separate figures. A MATLAB batch assertion checks figure names, titles, line lengths, and exact Y data.

**Tech Stack:** MATLAB R2022b script and batch-mode assertions.

## Global Constraints

- Modify `data0822/2/huizhiboxing.m` directly.
- Draw only channel 0, which is MATLAB row 1.
- Create two independent figures; do not overlay or align the captures.
- Titles are exactly `标准数据` and `SD数据`.
- Plot each capture's complete waveform against its own sample index.
- Resolve MAT paths from the script directory, not the current MATLAB directory.

---

### Task 1: Plot both channel 0 waveforms

**Files:**
- Modify: `data0822/2/huizhiboxing.m`
- Test: MATLAB R2022b batch assertion (no persistent test artifact)

**Interfaces:**
- Consumes: `202608221612.mat:data_all` and `run_1_segment_1.mat:data_SD`, both `128 × N uint16`
- Produces: one figure named/titled `标准数据` and one figure named/titled `SD数据`

- [ ] **Step 1: Run the failing figure assertion against the current script**

```matlab
set(groot, 'DefaultFigureVisible', 'off');
run('E:/ESP_IDF_File/ESP32S3_30M_ADC_STREAM_TO_SDMMC/data0822/2/huizhiboxing.m');
standard_figure = findobj(groot, 'Type', 'figure', 'Name', '标准数据');
sd_figure = findobj(groot, 'Type', 'figure', 'Name', 'SD数据');
assert(isscalar(standard_figure) && isscalar(sd_figure), ...
    'Expected two named waveform figures');
```

Run:

```powershell
& 'E:\Program Files\MATLAB\R2022b\bin\matlab.exe' -batch "set(groot,'DefaultFigureVisible','off'); run('E:/ESP_IDF_File/ESP32S3_30M_ADC_STREAM_TO_SDMMC/data0822/2/huizhiboxing.m'); assert(isscalar(findobj(groot,'Type','figure','Name','标准数据')) && isscalar(findobj(groot,'Type','figure','Name','SD数据')),'Expected two named waveform figures');"
```

Expected: FAIL with `Expected two named waveform figures` because the existing script creates no figures.

- [ ] **Step 2: Replace the empty script with the complete implementation**

```matlab
clc;
clear;
close all;

script_dir = fileparts(mfilename('fullpath'));
standard_file = fullfile(script_dir, '202608221612.mat');
sd_file = fullfile(script_dir, 'run_1_segment_1.mat');

standard = load(standard_file, 'data_all');
sd_capture = load(sd_file, 'data_SD');

assert(isfield(standard, 'data_all'), ...
    '标准 MAT 文件中没有变量 data_all。');
assert(ismatrix(standard.data_all) && size(standard.data_all, 1) >= 1, ...
    'data_all 不是有效的二维采集矩阵。');
assert(isfield(sd_capture, 'data_SD'), ...
    'SD MAT 文件中没有变量 data_SD。');
assert(ismatrix(sd_capture.data_SD) && size(sd_capture.data_SD, 1) >= 1, ...
    'data_SD 不是有效的二维采集矩阵。');

standard_channel0 = double(standard.data_all(1, :));
sd_channel0 = double(sd_capture.data_SD(1, :));

figure('Name', '标准数据', 'NumberTitle', 'off');
plot(1:numel(standard_channel0), standard_channel0, 'b-');
title('标准数据');
xlabel('采样点');
ylabel('ADC 原始值');
grid on;
axis tight;

figure('Name', 'SD数据', 'NumberTitle', 'off');
plot(1:numel(sd_channel0), sd_channel0, 'r-');
title('SD数据');
xlabel('采样点');
ylabel('ADC 原始值');
grid on;
axis tight;
```

- [ ] **Step 3: Run the full content assertion**

```matlab
set(groot, 'DefaultFigureVisible', 'off');
run('E:/ESP_IDF_File/ESP32S3_30M_ADC_STREAM_TO_SDMMC/data0822/2/huizhiboxing.m');
expected_standard = load('E:/ESP_IDF_File/ESP32S3_30M_ADC_STREAM_TO_SDMMC/data0822/2/202608221612.mat', 'data_all');
expected_sd = load('E:/ESP_IDF_File/ESP32S3_30M_ADC_STREAM_TO_SDMMC/data0822/2/run_1_segment_1.mat', 'data_SD');
standard_figure = findobj(groot, 'Type', 'figure', 'Name', '标准数据');
sd_figure = findobj(groot, 'Type', 'figure', 'Name', 'SD数据');
standard_line = findobj(standard_figure, 'Type', 'line');
sd_line = findobj(sd_figure, 'Type', 'line');
assert(strcmp(standard_figure.CurrentAxes.Title.String, '标准数据'));
assert(strcmp(sd_figure.CurrentAxes.Title.String, 'SD数据'));
assert(isequal(standard_line.YData, double(expected_standard.data_all(1, :))));
assert(isequal(sd_line.YData, double(expected_sd.data_SD(1, :))));
close all;
```

Run the block with MATLAB `-batch`. Expected: exit code 0 with no assertion failure.

- [ ] **Step 4: Commit the plotting script**

The experimental data directory is ignored, so add only this script explicitly:

```powershell
git add -f -- data0822/2/huizhiboxing.m
git commit -m "feat: plot channel 0 standard and SD waveforms"
```

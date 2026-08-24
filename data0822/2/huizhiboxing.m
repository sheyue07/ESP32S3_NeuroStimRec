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

% 通道 0 对应 128 行数据中的第 1 行。
standard_channel0 = double(standard.data_all(1, :));
sd_channel0 = double(sd_capture.data_SD(1, :));

figure('Name', '标准数据', 'NumberTitle', 'off');
plot(1:numel(standard_channel0), standard_channel0, 'b-');
title('标准数据', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('采样点');
ylabel('ADC 原始值');
grid on;
axis tight;

figure('Name', 'SD数据', 'NumberTitle', 'off');
plot(1:numel(sd_channel0), sd_channel0, 'r-');
title('SD数据', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('采样点');
ylabel('ADC 原始值');
grid on;
axis tight;

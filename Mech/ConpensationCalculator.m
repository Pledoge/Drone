%% Pitch轴重力补偿拟合 (角度 vs 电流) - 一次函数拟合
clear; clc; close all;

%% 读取数据
data = readtable('Ozone_DataSampling_260310_gravity.csv');

col_names = data.Properties.VariableNames;
pitch_col = '';
current_col = '';
for i = 1:length(col_names)
    name = col_names{i};
    if contains(name, 'pitch', 'IgnoreCase', true)
        pitch_col = name;
    end
    if contains(name, 'currentSet', 'IgnoreCase', true) || contains(name, 'Set', 'IgnoreCase', true)
        current_col = name;
    end
end

if isempty(pitch_col) || isempty(current_col)
    pitch_col = col_names{5};
    current_col = col_names{4};
    fprintf('自动匹配列名失败，使用第4列(%s)和第5列(%s)\n', current_col, pitch_col);
end

pitch = data.(pitch_col);
current_set = data.(current_col);

%% 去除无效数据
valid = isfinite(pitch) & isfinite(current_set);
pitch = pitch(valid);
current_set = current_set(valid);

if isempty(pitch)
    error('没有有效数据');
end

%% 限制角度范围 -0.2 到 0.6
range_idx = (pitch >= -0.2) & (pitch <= 0.6);
pitch = pitch(range_idx);
current_set = current_set(range_idx);

fprintf('限制角度范围 [-0.2, 0.6] rad 后剩余: %d 个点\n', length(pitch));

if isempty(pitch)
    error('角度范围内没有有效数据');
end

%% 过滤跳变点：基于滑动窗口中位数，去除偏差超过阈值的点
window_size = 11;
pitch_med = movmedian(pitch, window_size);
current_med = movmedian(current_set, window_size);

pitch_dev = abs(pitch - pitch_med);
current_dev = abs(current_set - current_med);

pitch_threshold = 3 * std(pitch_dev);
current_threshold = 3 * std(current_dev);

inlier_idx = (pitch_dev < pitch_threshold) & (current_dev < current_threshold);

pitch_clean = pitch(inlier_idx);
current_clean = current_set(inlier_idx);

fprintf('过滤前: %d 个点, 过滤后: %d 个点, 去除: %d 个跳变点\n', ...
    length(pitch), length(pitch_clean), length(pitch) - length(pitch_clean));

%% 去重取平均
[pitch_unique, ~, ic] = unique(round(pitch_clean, 6));
current_avg = accumarray(ic, current_clean, [], @mean);

%% 一次函数拟合
p = polyfit(pitch_unique, current_avg, 1);
k = p(1);  % 斜率
b = p(2);  % 截距

y_fit = polyval(p, pitch_unique);
SSres = sum((current_avg - y_fit).^2);
SStot = sum((current_avg - mean(current_avg)).^2);
if SStot == 0
    R2 = 1;
else
    R2 = 1 - SSres / SStot;
end

fprintf('\n拟合结果: current = %.10f * pitch + %.10f\n', k, b);
fprintf('R? = %.10f\n', R2);

%% 绘图
figure('Name', 'Pitch角度 vs 设定电流');
scatter(pitch_unique, current_avg, 30, 'b', 'filled'); hold on;

pitch_plot = linspace(-0.2, 0.6, 500);
current_plot = polyval(p, pitch_plot);
plot(pitch_plot, current_plot, 'r-', 'LineWidth', 2);

xlabel('Pitch角度 (rad)');
ylabel('设定电流 (A)');
title(sprintf('重力补偿拟合: I = %.6f * pitch + %.6f (R?=%.6f)', k, b, R2));
legend('数据点(过滤后)', '一次拟合');
grid on;

%% 生成C语言头文件
c_filename = 'pitch_gravity_compensation.h';
fid = fopen(c_filename, 'w');

fprintf(fid, '/**\n');
fprintf(fid, ' * @brief  Pitch轴重力补偿函数 (自动生成)\n');
fprintf(fid, ' * @note   一次函数拟合, R? = %.10f\n', R2);
fprintf(fid, ' *         current = k * pitch + b\n');
fprintf(fid, ' *         k = %.15f\n', k);
fprintf(fid, ' *         b = %.15f\n', b);
fprintf(fid, ' *         Pitch范围: [-0.200000, 0.600000] rad\n');
fprintf(fid, ' *         低于水平面为正\n');
fprintf(fid, ' */\n\n');

fprintf(fid, '#pragma once\n\n');

fprintf(fid, '#define PITCH_GRAV_K  (%.15ef)\n', k);
fprintf(fid, '#define PITCH_GRAV_B  (%.15ef)\n\n', b);

fprintf(fid, '/**\n');
fprintf(fid, ' * @brief  根据pitch角度计算重力补偿电流\n');
fprintf(fid, ' * @param  pitch 当前pitch角度 (rad), 低于水平面为正\n');
fprintf(fid, ' * @return 重力补偿电流 (A)\n');
fprintf(fid, ' */\n');
fprintf(fid, 'static inline float pitchGravityCompensation(float pitch)\n');
fprintf(fid, '{\n');
fprintf(fid, '    return %.15ef * pitch + %.15ef;\n', k, b);
fprintf(fid, '}\n');

fclose(fid);
fprintf('\nC代码已生成: %s\n', c_filename);
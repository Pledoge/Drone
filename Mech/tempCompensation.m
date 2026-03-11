%% Pitch 重力补偿拟合
% 从 Ozone 采样的 CSV 提取 pitch 角度 vs 电机输出，拟合补偿函数
% 第3列: pitch 角度 (rad)，正数表示低于水平
% 第4列: 电机输出电流

clear; clc; close all;

%% 1. 读取 CSV 数据
data = readtable('Ozone_DataSampling_260305.csv');
pitch_angle = data{:, 3};   % 第3列：pitch角度 (rad)
motor_output = data{:, 4};  % 第4列：电机输出电流

%% 2. 数据预处理：按角度分 bin 取均值，减少噪声
% 去除无效数据（角度和输出都不为0的点）
valid_idx = (pitch_angle ~= 0) | (motor_output ~= 0);
pitch_angle = pitch_angle(valid_idx);
motor_output = motor_output(valid_idx);

% 按角度排序
[pitch_angle, sort_idx] = sort(pitch_angle);
motor_output = motor_output(sort_idx);

% 分 bin 取均值（每个 bin 宽度约 0.005 rad）
bin_width = 0.005;
angle_min = floor(min(pitch_angle) / bin_width) * bin_width;
angle_max = ceil(max(pitch_angle) / bin_width) * bin_width;
bin_edges = angle_min:bin_width:angle_max;
bin_centers = bin_edges(1:end-1) + bin_width / 2;

avg_output = zeros(size(bin_centers));
for i = 1:length(bin_centers)
    idx = (pitch_angle >= bin_edges(i)) & (pitch_angle < bin_edges(i+1));
    if sum(idx) > 10  % 至少10个点才有效
        avg_output(i) = mean(motor_output(idx));
    else
        avg_output(i) = NaN;
    end
end

% 去除 NaN
valid = ~isnan(avg_output);
bin_centers = bin_centers(valid);
avg_output = avg_output(valid);

%% 3. 选择拟合函数
% 重力补偿与 sin(pitch) 成正比：compensation = a * sin(pitch) + b * cos(pitch) + c
% 对于小角度也可以用多项式，但 sin 模型物理意义更明确

% 方法1: 物理模型 f(x) = a*sin(x) + b*cos(x) + c
ft_sin = fittype('a*sin(x) + b*cos(x) + c', 'independent', 'x');
[fit_sin, gof_sin] = fit(bin_centers', avg_output', ft_sin, ...
    'StartPoint', [-1, 0, 0]);

% 方法2: 多项式拟合 (3阶)
[p3, S3] = polyfit(bin_centers, avg_output, 3);
[~, delta3] = polyval(p3, bin_centers, S3);
rmse_poly3 = sqrt(mean(delta3.^2));

% 方法3: 多项式拟合 (2阶)
[p2, S2] = polyfit(bin_centers, avg_output, 2);
[~, delta2] = polyval(p2, bin_centers, S2);
rmse_poly2 = sqrt(mean(delta2.^2));

fprintf('=== 拟合结果比较 ===\n');
fprintf('sin/cos 模型: R? = %.6f, RMSE = %.6f\n', gof_sin.rsquare, gof_sin.rmse);
fprintf('  a = %.8f, b = %.8f, c = %.8f\n', fit_sin.a, fit_sin.b, fit_sin.c);
fprintf('3阶多项式:    RMSE = %.6f\n', rmse_poly3);
fprintf('  p = [%.8f, %.8f, %.8f, %.8f]\n', p3(1), p3(2), p3(3), p3(4));
fprintf('2阶多项式:    RMSE = %.6f\n', rmse_poly2);
fprintf('  p = [%.8f, %.8f, %.8f]\n', p2(1), p2(2), p2(3));

%% 4. 绘图对比
figure('Position', [100, 100, 1200, 500]);

% 子图1: 原始数据 + 分 bin 均值
subplot(1, 2, 1);
plot(pitch_angle(1:100:end), motor_output(1:100:end), '.', 'Color', [0.8, 0.8, 0.8], 'MarkerSize', 1);
hold on;
plot(bin_centers, avg_output, 'ko', 'MarkerSize', 6, 'LineWidth', 1.5);
xlabel('Pitch 角度 (rad)');
ylabel('电机输出');
title('原始数据 & 分 bin 均值');
legend('原始数据 (下采样)', '分 bin 均值', 'Location', 'best');
grid on;

% 子图2: 拟合对比
subplot(1, 2, 2);
plot(bin_centers, avg_output, 'ko', 'MarkerSize', 6, 'LineWidth', 1.5);
hold on;

x_fit = linspace(min(bin_centers), max(bin_centers), 200);
plot(x_fit, feval(fit_sin, x_fit), 'r-', 'LineWidth', 2);
plot(x_fit, polyval(p3, x_fit), 'b--', 'LineWidth', 1.5);
plot(x_fit, polyval(p2, x_fit), 'g-.', 'LineWidth', 1.5);

xlabel('Pitch 角度 (rad)');
ylabel('电机输出');
title('拟合对比');
legend('数据', 'a·sin + b·cos + c', '3阶多项式', '2阶多项式', 'Location', 'best');
grid on;

sgtitle('Pitch 重力补偿拟合');

%% 5. 选择最佳模型并生成 C 代码
% 优先使用 sin/cos 物理模型（物理意义明确，外推性好）
% 若 R? 差距大则考虑多项式

fprintf('\n=== 生成 C 语言代码 ===\n\n');

% ---- sin/cos 模型 C 代码 ----
fprintf('// 方法1: 物理模型 (推荐)\n');
fprintf('// f(pitch) = a*sinf(pitch) + b*cosf(pitch) + c\n');
fprintf('// R? = %.6f, RMSE = %.6f\n', gof_sin.rsquare, gof_sin.rmse);
fprintf('// pitch: 云台相对水平的倾角 (rad), 正数表示低于水平\n');
fprintf('// 返回值: 重力补偿前馈电流\n');
fprintf('inline float pitchGravityCompensation(float pitch)\n');
fprintf('{\n');
fprintf('    return %.8ff * sinf(pitch) + %.8ff * cosf(pitch) + %.8ff;\n', ...
    fit_sin.a, fit_sin.b, fit_sin.c);
fprintf('}\n\n');

% ---- 3阶多项式 C 代码 ----
fprintf('// 方法2: 3阶多项式 (无需 sinf/cosf，更快)\n');
fprintf('// f(pitch) = p0*x? + p1*x? + p2*x + p3\n');
fprintf('inline float pitchGravityCompensationPoly(float pitch)\n');
fprintf('{\n');
fprintf('    return ((%.8ff * pitch + %.8ff) * pitch + %.8ff) * pitch + %.8ff;\n', ...
    p3(1), p3(2), p3(3), p3(4));
fprintf('}\n');

fprintf('\n完成！请将上述 C 函数复制到项目中使用。\n');

%% 摩擦力补偿拟合
clear; clc; close all;

%% 读取数据
data = readtable('Ozone_DataSampling_260310.csv');
time = data.Time;
pos_output = data.positive_output;
neg_output = data.negative_output;
pos_speed = data.positive_speed;
neg_speed = data.negative_speed;

%% 提取有效数据（非零部分）
pos_valid_idx = (pos_speed ~= 0) | (pos_output ~= 0);
neg_valid_idx = (neg_speed ~= 0) | (neg_output ~= 0);

pos_speed_valid = pos_speed(pos_valid_idx);
pos_output_valid = pos_output(pos_valid_idx);
neg_speed_valid = neg_speed(neg_valid_idx);
neg_output_valid = neg_output(neg_valid_idx);

%% 判断匀速部分：速度变化率小于阈值
% 正向
if length(pos_speed_valid) > 2
    pos_time_valid = time(pos_valid_idx);
    pos_accel = gradient(pos_speed_valid) ./ gradient(pos_time_valid);
    accel_threshold = std(pos_accel) * 0.5;
    pos_steady_idx = abs(pos_accel) < accel_threshold;
    pos_speed_steady = pos_speed_valid(pos_steady_idx);
    pos_output_steady = pos_output_valid(pos_steady_idx);
else
    pos_speed_steady = pos_speed_valid;
    pos_output_steady = pos_output_valid;
end

% 负向
if length(neg_speed_valid) > 2
    neg_time_valid = time(neg_valid_idx);
    neg_accel = gradient(neg_speed_valid) ./ gradient(neg_time_valid);
    accel_threshold_neg = std(neg_accel) * 0.5;
    neg_steady_idx = abs(neg_accel) < accel_threshold_neg;
    neg_speed_steady = neg_speed_valid(neg_steady_idx);
    neg_output_steady = neg_output_valid(neg_steady_idx);
else
    neg_speed_steady = neg_speed_valid;
    neg_output_steady = neg_output_valid;
end

%% 合并正负数据
all_speed = [pos_speed_steady; neg_speed_steady];
all_output = [pos_output_steady; neg_output_steady];

% 去除NaN和Inf
valid = isfinite(all_speed) & isfinite(all_output);
all_speed = all_speed(valid);
all_output = all_output(valid);

if isempty(all_speed)
    error('没有有效的匀速数据，请检查CSV数据是否全为0');
end

%% 拟合：output = a * speed + b * sign(speed)
% 模型: f(v) = a*v + b*sign(v)
% 即 粘性摩擦 + 库仑摩擦
X = [all_speed, sign(all_speed)];
coeffs = X \ all_output;
a = coeffs(1);  % 粘性摩擦系数
b = coeffs(2);  % 库仑摩擦力

fprintf('拟合结果:\n');
fprintf('  粘性摩擦系数 a = %.6f\n', a);
fprintf('  库仑摩擦力   b = %.6f\n', b);

%% 绘图
figure('Name', '摩擦力补偿拟合');
scatter(all_speed, all_output, 20, 'b', 'filled'); hold on;

speed_plot = linspace(min(all_speed), max(all_speed), 500);
output_plot = a * speed_plot + b * sign(speed_plot);
plot(speed_plot, output_plot, 'r-', 'LineWidth', 2);

xlabel('速度 (RPM)');
ylabel('输出 (电压/电流)');
title('摩擦力补偿拟合: output = a*speed + b*sign(speed)');
legend('匀速数据点', '拟合曲线');
grid on;

%% 也尝试三次多项式拟合（正负分开）
if length(pos_speed_steady) >= 2
    pos_poly = polyfit(pos_speed_steady, pos_output_steady, min(3, length(pos_speed_steady)-1));
else
    pos_poly = [0, 0];
end

if length(neg_speed_steady) >= 2
    neg_poly = polyfit(neg_speed_steady, neg_output_steady, min(3, length(neg_speed_steady)-1));
else
    neg_poly = [0, 0];
end

%% 生成C语言代码
c_filename = 'friction_compensation.h';
fid = fopen(c_filename, 'w');

fprintf(fid, '/**\n');
fprintf(fid, ' * @brief 摩擦力补偿函数 (自动生成)\n');
fprintf(fid, ' * @note  模型: output = a * speed + b * sign(speed)\n');
fprintf(fid, ' *        粘性摩擦系数 a = %.6f\n', a);
fprintf(fid, ' *        库仑摩擦力   b = %.6f\n', b);
fprintf(fid, ' */\n\n');

fprintf(fid, '#pragma once\n\n');
fprintf(fid, '#include <math.h>\n\n');

fprintf(fid, '#define FRICTION_VISCOUS_COEFF  (%.6ff)\n', a);
fprintf(fid, '#define FRICTION_COULOMB_FORCE  (%.6ff)\n\n', b);

fprintf(fid, '/**\n');
fprintf(fid, ' * @brief  根据目标速度计算摩擦力补偿输出\n');
fprintf(fid, ' * @param  speed 目标速度 (RPM)\n');
fprintf(fid, ' * @return 摩擦力补偿输出值\n');
fprintf(fid, ' */\n');
fprintf(fid, 'static inline float frictionCompensation(float speed)\n');
fprintf(fid, '{\n');
fprintf(fid, '    if (fabsf(speed) < 1e-3f)\n');
fprintf(fid, '        return 0.0f;\n');
fprintf(fid, '    float sign = (speed > 0.0f) ? 1.0f : -1.0f;\n');
fprintf(fid, '    return %.6ff * speed + %.6ff * sign;\n', a, b);
fprintf(fid, '}\n');

fclose(fid);
fprintf('\nC代码已生成: %s\n', c_filename);

%% 如果数据全为0，给出提示
if all(all_output == 0)
    warning('所有输出数据为0，拟合结果无意义。请确保CSV中包含有效的摩擦力测试数据。');
end
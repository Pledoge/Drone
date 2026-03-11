%% Pitch轴摩擦力+重力补偿拟合
clear; clc; close all;

%% 读取数据
data = readtable('Ozone_DataSampling_260310_pitch.csv');
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

%% 判断匀速部分
% 正向（顺重力）
if length(pos_speed_valid) > 2
    pos_time_valid = time(pos_valid_idx);
    pos_accel = gradient(pos_speed_valid) ./ gradient(pos_time_valid);
    pos_steady_idx = abs(pos_accel) < std(pos_accel) * 0.5;
    pos_speed_steady = pos_speed_valid(pos_steady_idx);
    pos_output_steady = pos_output_valid(pos_steady_idx);
else
    pos_speed_steady = pos_speed_valid;
    pos_output_steady = pos_output_valid;
end

% 负向（逆重力）
if length(neg_speed_valid) > 2
    neg_time_valid = time(neg_valid_idx);
    neg_accel = gradient(neg_speed_valid) ./ gradient(neg_time_valid);
    neg_steady_idx = abs(neg_accel) < std(neg_accel) * 0.5;
    neg_speed_steady = neg_speed_valid(neg_steady_idx);
    neg_output_steady = neg_output_valid(neg_steady_idx);
else
    neg_speed_steady = neg_speed_valid;
    neg_output_steady = neg_output_valid;
end

%% 去重：对相同速度取平均输出
if ~isempty(pos_speed_steady)
    [pos_speed_unique, ~, ic] = unique(pos_speed_steady);
    pos_output_unique = accumarray(ic, pos_output_steady, [], @mean);
else
    pos_speed_unique = [];
    pos_output_unique = [];
end

if ~isempty(neg_speed_steady)
    [neg_speed_unique, ~, ic] = unique(neg_speed_steady);
    neg_output_unique = accumarray(ic, neg_output_steady, [], @mean);
else
    neg_speed_unique = [];
    neg_output_unique = [];
end

%% 拟合模型
% 匀速时: output = friction + gravity (正向) 或 output = -friction + gravity (负向)
% 正向(顺重力, speed>0): output_pos = a * speed + b + g
%   其中 b = 库仑摩擦(正), g = 重力项
% 负向(逆重力, speed<0): output_neg = a * speed - b + g
%   其中 -b = 库仑摩擦(负方向摩擦反向), g = 重力项不变
%
% 综合模型: output = a * speed + b * sign(speed) + g
% 三个未知数: a(粘性摩擦), b(库仑摩擦), g(重力补偿)

all_speed = [pos_speed_unique; neg_speed_unique];
all_output = [pos_output_unique; neg_output_unique];

valid = isfinite(all_speed) & isfinite(all_output);
all_speed = all_speed(valid);
all_output = all_output(valid);

if isempty(all_speed)
    error('没有有效的匀速数据，请检查CSV数据');
end

% output = a * speed + b * sign(speed) + g
X = [all_speed, sign(all_speed), ones(size(all_speed))];
coeffs = X \ all_output;
a = coeffs(1);  % 粘性摩擦系数
b = coeffs(2);  % 库仑摩擦力
g = coeffs(3);  % 重力补偿项

fprintf('拟合结果:\n');
fprintf('  粘性摩擦系数 a = %.6f\n', a);
fprintf('  库仑摩擦力   b = %.6f\n', b);
fprintf('  重力补偿     g = %.6f\n', g);

%% 绘图
figure('Name', 'Pitch轴摩擦力+重力补偿拟合');

if ~isempty(pos_speed_unique)
    scatter(pos_speed_unique, pos_output_unique, 40, 'r', 'filled'); hold on;
end
if ~isempty(neg_speed_unique)
    scatter(neg_speed_unique, neg_output_unique, 40, 'b', 'filled'); hold on;
end

speed_range = linspace(min(all_speed)*1.1, max(all_speed)*1.1, 500);
output_fit = a * speed_range + b * sign(speed_range) + g;
plot(speed_range, output_fit, 'k-', 'LineWidth', 2);

% 单独画摩擦力和重力分量
friction_fit = a * speed_range + b * sign(speed_range);
plot(speed_range, friction_fit, 'g--', 'LineWidth', 1.5);
yline(g, 'm--', 'LineWidth', 1.5);

xlabel('速度 (RPM)');
ylabel('输出');
title('Pitch轴: output = a*speed + b*sign(speed) + g');
legend('正向(顺重力)数据', '负向(逆重力)数据', '总拟合', '摩擦力分量', '重力分量');
grid on;

%% 生成C语言代码
c_filename = 'pitch_friction_compensation.h';
fid = fopen(c_filename, 'w');

fprintf(fid, '/**\n');
fprintf(fid, ' * @brief Pitch轴摩擦力+重力补偿函数 (自动生成)\n');
fprintf(fid, ' * @note  模型: output = a * speed + b * sign(speed) + g\n');
fprintf(fid, ' *        粘性摩擦系数 a = %.6f\n', a);
fprintf(fid, ' *        库仑摩擦力   b = %.6f\n', b);
fprintf(fid, ' *        重力补偿     g = %.6f\n', g);
fprintf(fid, ' */\n\n');

fprintf(fid, '#pragma once\n\n');
fprintf(fid, '#include <math.h>\n\n');

fprintf(fid, '#define PITCH_FRICTION_VISCOUS_COEFF  (%.6ff)\n', a);
fprintf(fid, '#define PITCH_FRICTION_COULOMB_FORCE  (%.6ff)\n', b);
fprintf(fid, '#define PITCH_GRAVITY_COMPENSATION    (%.6ff)\n\n', g);

fprintf(fid, '/**\n');
fprintf(fid, ' * @brief  根据目标速度计算Pitch轴摩擦力补偿输出(含重力)\n');
fprintf(fid, ' * @param  speed 目标速度 (RPM)\n');
fprintf(fid, ' * @return 补偿输出值\n');
fprintf(fid, ' */\n');
fprintf(fid, 'static inline float pitchFrictionCompensation(float speed)\n');
fprintf(fid, '{\n');
fprintf(fid, '    float friction;\n');
fprintf(fid, '    if (fabsf(speed) < 1e-3f)\n');
fprintf(fid, '        friction = 0.0f;\n');
fprintf(fid, '    else\n');
fprintf(fid, '    {\n');
fprintf(fid, '        float sign = (speed > 0.0f) ? 1.0f : -1.0f;\n');
fprintf(fid, '        friction = %.6ff * speed + %.6ff * sign;\n', a, b);
fprintf(fid, '    }\n');
fprintf(fid, '    return friction + %.6ff;  // 摩擦力 + 重力补偿\n', g);
fprintf(fid, '}\n\n');

fprintf(fid, '/**\n');
fprintf(fid, ' * @brief  仅获取重力补偿值\n');
fprintf(fid, ' * @return 重力补偿输出值\n');
fprintf(fid, ' */\n');
fprintf(fid, 'static inline float pitchGravityCompensation(void)\n');
fprintf(fid, '{\n');
fprintf(fid, '    return %.6ff;\n', g);
fprintf(fid, '}\n');

fclose(fid);
fprintf('\nC代码已生成: %s\n', c_filename);

if length(unique(all_speed)) < 3
    warning('有效速度档位少于3个，拟合结果可能不可靠。建议增加更多不同速度的测试数据。');
end
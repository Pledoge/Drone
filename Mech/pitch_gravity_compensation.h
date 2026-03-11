/**
 * @brief  Pitch轴重力补偿函数 (自动生成)
 * @note   一次函数拟合, R? = 0.9079711211
 *         current = k * pitch + b
 *         k = 0.632601080547826
 *         b = -0.268619912583449
 *         Pitch范围: [-0.200000, 0.600000] rad
 *         低于水平面为正
 */

#pragma once

#define PITCH_GRAV_K  (6.326010805478256e-01f)
#define PITCH_GRAV_B  (-2.686199125834486e-01f)

/**
 * @brief  根据pitch角度计算重力补偿电流
 * @param  pitch 当前pitch角度 (rad), 低于水平面为正
 * @return 重力补偿电流 (A)
 */
static inline float pitchGravityCompensation(float pitch)
{
    return 6.326010805478256e-01f * pitch + -2.686199125834486e-01f;
}

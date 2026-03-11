/**
 * @brief 摩擦力补偿函数 (自动生成)
 * @note  模型: output = a * speed + b * sign(speed)
 *        粘性摩擦系数 a = -84.367143
 *        库仑摩擦力   b = -897.532363
 */

#pragma once

#include <math.h>

#define FRICTION_VISCOUS_COEFF  (-84.367143f)
#define FRICTION_COULOMB_FORCE  (-897.532363f)

/**
 * @brief  根据目标速度计算摩擦力补偿输出
 * @param  speed 目标速度 (RPM)
 * @return 摩擦力补偿输出值
 */
static inline float frictionCompensation(float speed)
{
    if (fabsf(speed) < 1e-3f)
        return 0.0f;
    float sign = (speed > 0.0f) ? 1.0f : -1.0f;
    return -84.367143f * speed + -897.532363f * sign;
}

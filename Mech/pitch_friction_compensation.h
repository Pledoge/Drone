/**
 * @brief Pitch轴摩擦力+重力补偿函数 (自动生成)
 * @note  模型: output = a * speed + b * sign(speed) + g
 *        粘性摩擦系数 a = -0.009184
 *        库仑摩擦力   b = 0.370828
 *        重力补偿     g = -0.006264
 */

#pragma once

#include <math.h>

#define PITCH_FRICTION_VISCOUS_COEFF  (-0.009184f)
#define PITCH_FRICTION_COULOMB_FORCE  (0.370828f)
#define PITCH_GRAVITY_COMPENSATION    (-0.006264f)

/**
 * @brief  根据目标速度计算Pitch轴摩擦力补偿输出(含重力)
 * @param  speed 目标速度 (RPM)
 * @return 补偿输出值
 */
static inline float pitchFrictionCompensation(float speed)
{
    float friction;
    if (fabsf(speed) < 1e-3f)
        friction = 0.0f;
    else
    {
        float sign = (speed > 0.0f) ? 1.0f : -1.0f;
        friction = -0.009184f * speed + 0.370828f * sign;
    }
    return friction + -0.006264f;  // 摩擦力 + 重力补偿
}

/**
 * @brief  仅获取重力补偿值
 * @return 重力补偿输出值
 */
static inline float pitchGravityCompensation(void)
{
    return -0.006264f;
}

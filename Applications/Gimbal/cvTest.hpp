#pragma once

#include "FreeRTOS.h"
#include "task.h"
struct YawTestConfig
{
    float minAngle               = -0.02f;  // Minimum yaw angle (radians)
    float maxAngle               = 0.58f;   // Maximum yaw angle (radians)
    float rate                   = 0.2f;    // Rate of change (rad/s)
    float currentAngle           = 0.0f;    // Current angle (radians)
    TickType_t lastUpdateTime    = 0;       // Last update time in milliseconds
    TickType_t cdStartTime       = 0;
    const TickType_t CD_DURATION = 147;  // Cooldown duration in milliseconds
    bool flag                    = false;
} yawTestConfig;
float generateYawTestSignal(YawTestConfig &config, uint32_t timeMs)
{
    float dt              = (float)(timeMs - config.lastUpdateTime) / 1000.0f;
    config.lastUpdateTime = timeMs;

    if (config.currentAngle >= config.maxAngle)
    {
        if (config.flag == false)
        {
            config.cdStartTime = timeMs;
            config.flag        = true;
        }
        else if (timeMs - config.cdStartTime > config.CD_DURATION)
        {
            config.currentAngle = config.minAngle;
            config.flag         = false;
        }
    }
    else
    {
        config.currentAngle += config.rate * dt;
    }

    return config.currentAngle;
}
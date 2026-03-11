#include "ShooterController.hpp"

#include "task.h"

namespace Task
{
namespace Shooter
{

bool ShooterController::checkShootingState()
{
    return fabs(left_shooter_motor_->getRPMFeedback() - FRICTION_WHEEL_TARGET_RPM) < 400.0f &&
           fabs(right_shooter_motor_->getRPMFeedback() - FRICTION_WHEEL_TARGET_RPM) < 400.0f;
}

void ShooterController::stuckChecker()
{
    static uint16_t loader_stuck_count = 0;
    float loader_current               = loader_motor_->getCurrentFeedback();

    if (fabs(loader_current) > LOADER_STUCK_CURRENT_THRESHOLD && fabs(loader_motor_->getRPMFeedback()) < LOADER_STUCK_RPM_THRESHOLD)
    {
        loader_stuck_count++;
    }
    else
    {
        loader_stuck_count = 0;
    }

    if (loader_stuck_count > LOADER_STUCK_COUNT_THRESHOLD)
    {
        loader_stuck_count = 0;
        is_stuck_          = true;
        last_stuck_angle_  = loader_motor_->getAccumulatedPosition();
    }
}

void ShooterController::stuckHandler()
{
    float target_position          = last_stuck_angle_ - LOADER_STUCK_BACK_ANGLE;
    static uint16_t cooldown_count = 0;
    static bool in_cooldown        = false;
    static int timeout_count       = 0;
    if (!in_cooldown)
        loader_motor_->setCurrent(-4.0f);
    if (loader_motor_->getAccumulatedPosition() <= target_position + LOADER_RESTORE_ANGLE_LIMIT || timeout_count > LOADER_RESTORE_TIME_LIMIT)
    {
        in_cooldown   = true;
        timeout_count = 0;
    }
    else if (!in_cooldown)
    {
        timeout_count++;
    }
    if (in_cooldown)
    {
        loader_motor_->setCurrent(0.0f);
        cooldown_count++;
        if (cooldown_count >= LOADER_STUCK_COOLDOWN)
        {
            cooldown_count = 0;
            in_cooldown    = false;
            is_stuck_      = false;
        }
    }
}

void ShooterController::updateShooterState(bool _shoot_command, bool _load_command)
{
    switch (shooter_state_)
    {
    case ShooterState::IDLE:
    {
        if (_shoot_command)
        {
            shooter_state_ = ShooterState::SHOOTING;
        }
        break;
    }
    case ShooterState::SHOOTING:
    {
        if (_load_command)
        {
            shooter_state_ = ShooterState::LOADING;
        }
        else if (!_shoot_command)
        {
            shooter_state_ = ShooterState::IDLE;
        }
        break;
    }
    case ShooterState::LOADING:
    {
        if (is_stuck_)
        {
            shooter_state_ = ShooterState::STUCK;
        }
        if (!_load_command)
        {
            if (_shoot_command)
            {
                shooter_state_ = ShooterState::SHOOTING;
            }
            else
            {
                shooter_state_ = ShooterState::IDLE;
            }
        }
        break;
    }
    case ShooterState::STUCK:
    {
        if (!is_stuck_)
        {
            if (_shoot_command)
            {
                shooter_state_ = ShooterState::SHOOTING;
            }
            else if (_load_command)
            {
                shooter_state_ = ShooterState::LOADING;
            }
            else
            {
                shooter_state_ = ShooterState::IDLE;
            }
        }
        break;
    }
    default:
    {
        shooter_state_ = ShooterState::IDLE;
        break;
    }
    }
}

void ShooterController::updateAccRadio(bool _is_shooting)
{
    if (_is_shooting)
    {
        return;
    }
    acc_radio_ = left_shooter_motor_->getRPMFeedback() > right_shooter_motor_->getRPMFeedback()
                     ? left_shooter_motor_->getRPMFeedback() / FRICTION_WHEEL_TARGET_RPM
                     : right_shooter_motor_->getRPMFeedback() / FRICTION_WHEEL_TARGET_RPM;
}

void ShooterController::triggerShooter()
{
    if (disable_all_)
    {
        disableShooter();
        return;
    }
    static uint32_t last_update_time = 0;
    float target_rpm                 = FRICTION_WHEEL_TARGET_RPM * acc_radio_;
    if (xTaskGetTickCount() - last_update_time > pdMS_TO_TICKS(50))
    {
        if (acc_radio_ < 0.0f)
        {
            acc_radio_ = 0.0f;  // Reset the acceleration radio if it is negative
        }
        acc_radio_ + 0.01f >= 1.0f ? acc_radio_ = 1.0f : acc_radio_ += 0.01f;
        last_update_time = xTaskGetTickCount();
    }
    left_shooter_motor_->setCurrent((*left_shooter_pid_)(target_rpm, left_shooter_motor_->getRPMFeedback()));
    right_shooter_motor_->setCurrent((*right_shooter_pid_)(target_rpm, right_shooter_motor_->getRPMFeedback()));
}

void ShooterController::disableShooter()
{
    left_shooter_motor_->setCurrent(0.0f);
    right_shooter_motor_->setCurrent(0.0f);
}

void ShooterController::stopLoader()
{
    if (disable_all_)
    {
        disableLoader();
        return;
    }
    loader_motor_->setCurrent((*loader_pid_)(loader_motor_->getAccumulatedPosition(), loader_motor_->getAccumulatedPosition()));
}

void ShooterController::disableLoader()
{
    // loader_pid_->reset();
    loader_motor_->setCurrent(0.0f);
}

void ShooterController::triggerLoader()
{
    // static uint32_t last_awake_time = xTaskGetTickCount();

    loader_motor_->setCurrent((*loader_pid_)(LOADER_SPEED, loader_motor_->getRPMFeedback()));
}

}  // namespace Shooter
}  // namespace Task
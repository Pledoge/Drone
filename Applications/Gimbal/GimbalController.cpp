#include "GimbalController.hpp"

#include "CommandTask.hpp"
#include "Config.hpp"
#include "IMU.hpp"

namespace Task
{
namespace Gimbal
{
#define EDGE_LIMIT 0.1f

Core::Control::Gimbal::IMUDatas imu_data;

inline float getPitchCompensation(float pitch_rad) { return 0.6326010805f * pitch_rad + -0.2686199126f; }

static inline float frictionCompensation(float speed)
{
    if (fabsf(speed) < 1e-3f)
        return 0.0f;
    float sign = (speed > 0.0f) ? 1.0f : -1.0f;
    return -84.367143f * speed + -897.532363f * sign;
}

void GimbalController::updateGimbalAttitude()
{
    imu_data.imuInstall        = Core::Control::Gimbal::IMUDatas::IMUInstall::PITCH;
    imu_data.IMUQuat           = Core::Drivers::IMU::getQuaternion();
    current_motor_euler_.pitch = pitch_motor_->getPositionFeedback();
    current_motor_euler_.yaw   = yaw_motor_->getPositionFeedback();
    current_motor_euler_.roll  = 0.0f;

    current_gimbal_quat_  = imu_data.IMUQuat;
    current_gimbal_euler_ = Core::Control::Gimbal::quatToPitchYaw(current_gimbal_quat_);
}

void GimbalController::updateTargetGimbalPos(float _yaw_param, float _pitch_param, bool _target_mode)
{
    if (_target_mode)
    {
        changeTargetPosByTargetCommand(_yaw_param, _pitch_param);
    }
    else
    {
        changeTargetPosByCommand(_yaw_param, _pitch_param);
    }
    updateGimbalAttitude();
    target_gimbal_euler_.yaw   = Core::Utils::Math::wrapMinMax(target_gimbal_euler_.yaw, -M_PI, M_PI);
    target_gimbal_euler_.pitch = Core::Utils::Math::clamp<float>(target_gimbal_euler_.pitch, (float)M_PI_2);
    target_motor_euler_        = Core::Control::Gimbal::quatToPitchYaw(
        Core::Control::Gimbal::getChassisQuat(imu_data, current_motor_euler_).conjugate() *
        Core::Utils::Math::eulerZYXtoQuat<float>(target_gimbal_euler_.yaw, target_gimbal_euler_.pitch, target_gimbal_euler_.roll));

    float delta_yaw        = Core::Control::Gimbal::clampGimbalError(target_motor_euler_.yaw - current_motor_euler_.yaw);
    float delta_pitch      = Core::Control::Gimbal::clampGimbalError(target_motor_euler_.pitch - current_motor_euler_.pitch);
    float diff_min_yaw     = fabsf(Core::Control::Gimbal::clampGimbalError(current_motor_euler_.yaw - MIN_YAW_ANGLE));
    float diff_max_yaw     = fabsf(Core::Control::Gimbal::clampGimbalError(current_motor_euler_.yaw - MAX_YAW_ANGLE));
    float minimum_diff_yaw = fabsf(diff_min_yaw) < fabsf(diff_max_yaw) ? diff_min_yaw : diff_max_yaw;
    if (current_motor_euler_.yaw < MIN_YAW_ANGLE || current_motor_euler_.yaw > MAX_YAW_ANGLE)
    {
        if (minimum_diff_yaw < EDGE_LIMIT)
        {
        }
        else if (diff_min_yaw < diff_max_yaw)
        {
            target_gimbal_euler_.yaw = current_gimbal_euler_.yaw + MIN_YAW_ANGLE - current_motor_euler_.yaw;
        }
        else if (diff_min_yaw >= diff_max_yaw)
        {
            target_gimbal_euler_.yaw = current_gimbal_euler_.yaw + MAX_YAW_ANGLE - current_motor_euler_.yaw;
        }
    }
    else if (target_motor_euler_.yaw > MAX_YAW_ANGLE && diff_min_yaw > diff_max_yaw)
    {
        if (delta_yaw < 0.0f)
        {
            target_gimbal_euler_.yaw = current_gimbal_euler_.yaw + (MAX_YAW_ANGLE - current_motor_euler_.yaw) + (-0.01f);
        }
        else
        {
            target_gimbal_euler_.yaw = current_gimbal_euler_.yaw + (MAX_YAW_ANGLE - current_motor_euler_.yaw);
        }
    }
    else if (target_motor_euler_.yaw < MIN_YAW_ANGLE && diff_min_yaw < diff_max_yaw)
    {
        if (delta_yaw > 0.0f)
        {
            target_gimbal_euler_.yaw = current_gimbal_euler_.yaw + (MIN_YAW_ANGLE - current_motor_euler_.yaw) + 0.01f;
        }
        else
        {
            target_gimbal_euler_.yaw = current_gimbal_euler_.yaw + (MIN_YAW_ANGLE - current_motor_euler_.yaw);
        }
    }
    float diff_min_pitch     = fabsf(Core::Control::Gimbal::clampGimbalError(current_motor_euler_.pitch - MIN_PITCH_ANGLE));
    float diff_max_pitch     = fabsf(Core::Control::Gimbal::clampGimbalError(current_motor_euler_.pitch - MAX_PITCH_ANGLE));
    float minimum_diff_pitch = fabsf(diff_min_pitch) < fabsf(diff_max_pitch) ? diff_min_pitch : diff_max_pitch;
    if (current_motor_euler_.pitch < MIN_PITCH_ANGLE || current_motor_euler_.pitch > MAX_PITCH_ANGLE)
    {
        if (minimum_diff_pitch < EDGE_LIMIT && !_target_mode)
        {
        }
        else if (diff_min_pitch < diff_max_pitch)
        {
            target_gimbal_euler_.pitch = current_gimbal_euler_.pitch + MIN_PITCH_ANGLE - current_motor_euler_.pitch;
        }
        else if (diff_min_pitch >= diff_max_pitch)
        {
            target_gimbal_euler_.pitch = current_gimbal_euler_.pitch + MAX_PITCH_ANGLE - current_motor_euler_.pitch;
        }
    }
    else if (target_motor_euler_.pitch > MAX_PITCH_ANGLE && diff_min_pitch > diff_max_pitch)
    {
        // recalcuate the target gimbal pitch
        if (delta_pitch < 0.0f && !_target_mode)
        {
            target_gimbal_euler_.pitch = current_gimbal_euler_.pitch + (MAX_PITCH_ANGLE - current_motor_euler_.pitch) + (-0.01f);
        }
        else
        {
            target_gimbal_euler_.pitch = current_gimbal_euler_.pitch + (MAX_PITCH_ANGLE - current_motor_euler_.pitch);
        }
    }
    else if (target_motor_euler_.pitch < MIN_PITCH_ANGLE && diff_min_pitch < diff_max_pitch)
    {
        if (delta_pitch > 0.0f && !_target_mode)
        {
            target_gimbal_euler_.pitch = current_gimbal_euler_.pitch + (MIN_PITCH_ANGLE - current_motor_euler_.pitch) + 0.01f;
        }
        else
        {
            target_gimbal_euler_.pitch = current_gimbal_euler_.pitch + (MIN_PITCH_ANGLE - current_motor_euler_.pitch);
        }
    }

    angle_error_ = Core::Control::Gimbal::getCurrentGimbalError(imu_data, current_motor_euler_, target_gimbal_euler_);
}

void GimbalController::changeTargetPosByCommand(float _delta_yaw, float _delta_pitch)
{
    target_gimbal_euler_.yaw += _delta_yaw;
    target_gimbal_euler_.pitch += _delta_pitch;
}

void GimbalController::changeTargetPosByTargetCommand(float _yaw_param, float _pitch_param)
{
    target_gimbal_euler_.yaw   = _yaw_param;
    target_gimbal_euler_.pitch = _pitch_param;
}
void GimbalController::syncYawPosition()
{
    target_gimbal_euler_.yaw = current_gimbal_euler_.yaw;
    // angle_error_.yaw         = 0.0f;
}

void GimbalController::syncPitchPosition()
{
    target_gimbal_euler_.pitch = current_gimbal_euler_.pitch;
    // angle_error_.pitch         = 0.0f;
}

void GimbalController::setYawMotorOutput(bool _disable)
{
    if (_disable)
    {
        yaw_pos_pid_->resetIOut();
        yaw_motor_->setOutput(0.0f);
        return;
    }
    yaw_auto_aim_pid_->resetIOut();
    (*yaw_auto_aim_pid_)(yaw_motor_->getAccumulatedPosition() + angle_error_.yaw, yaw_motor_->getAccumulatedPosition());
    yaw_motor_->setOutput(frictionCompensation(yaw_motor_->getRPMFeedback()) +
                          (*yaw_pos_pid_)(yaw_motor_->getAccumulatedPosition() + angle_error_.yaw, yaw_motor_->getAccumulatedPosition()));
    // yaw_motor_->setCurrent(0);
}

void GimbalController::setPitchMotorOutput(bool _disable)
{
    if (_disable)
    {
        pitch_pos_pid_->resetIOut();
        pitch_motor_->setCurrent(0.0f);
        return;
    }
    pitch_auto_aim_pid_->resetIOut();
    (*pitch_auto_aim_pid_)(pitch_motor_->getAccumulatedPosition() + angle_error_.pitch, pitch_motor_->getAccumulatedPosition());
    pitch_motor_->setCurrent(getPitchCompensation(current_gimbal_euler_.pitch) +
                             (*pitch_pos_pid_)(pitch_motor_->getAccumulatedPosition() + angle_error_.pitch, pitch_motor_->getAccumulatedPosition()));
}
void GimbalController::setYawAutoAimOutput(bool _disable)
{
    if (_disable)
    {
        yaw_auto_aim_pid_->reset();
        yaw_motor_->setOutput(0.0f);
        return;
    }
    yaw_pos_pid_->resetIOut();
    (*yaw_pos_pid_)(yaw_motor_->getAccumulatedPosition() + angle_error_.yaw, yaw_motor_->getAccumulatedPosition());
    yaw_motor_->setOutput(frictionCompensation(yaw_motor_->getRPMFeedback()) +
                          (*yaw_auto_aim_pid_)(yaw_motor_->getAccumulatedPosition() + angle_error_.yaw, yaw_motor_->getAccumulatedPosition()));
}

void GimbalController::setPitchAutoAimOutput(bool _disable)
{
    if (_disable)
    {
        pitch_auto_aim_pid_->reset();
        pitch_motor_->setCurrent(0.0f);
        return;
    }
    pitch_pos_pid_->resetIOut();
    (*pitch_pos_pid_)(pitch_motor_->getAccumulatedPosition() + angle_error_.pitch, pitch_motor_->getAccumulatedPosition());
    pitch_motor_->setCurrent(
        getPitchCompensation(current_gimbal_euler_.pitch) +
        (*pitch_auto_aim_pid_)(pitch_motor_->getAccumulatedPosition() + angle_error_.pitch, pitch_motor_->getAccumulatedPosition()));
}

GimbalController::GimbalState GimbalController::getGimbalState() const { return gimbal_state_; }

void GimbalController::gimbalStateUpdate()
{
    switch (gimbal_state_)
    {
    case GimbalState::NORMAL:
    {
        if (disable_all_)
        {
            gimbal_state_ = GimbalState::STALL;
        }
        else if (is_compensating_ && getYawConnected() && getPitchConnected())
        {
            gimbal_state_ = GimbalState::COMPENSATING;
        }
        break;
    }
    case GimbalState::STALL:
    {
        if (disable_all_)
        {
            break;
        }
        if (!is_compensating_)
        {
            gimbal_state_ = GimbalState::NORMAL;
        }
        else if (getYawConnected() && getPitchConnected() && is_compensating_)
        {
            gimbal_state_ = GimbalState::COMPENSATING;
        }
        break;
    }
    case GimbalState::COMPENSATING:
    {
        if (disable_all_)
        {
            gimbal_state_ = GimbalState::STALL;
            break;
        }
        if (!is_compensating_)
        {
            gimbal_state_ = GimbalState::NORMAL;
        }
        break;
    }
    default:
        break;
    }
}

Core::Utils::Math::Quaternion<float> getCameraQ() { return imu_data.IMUQuat; }
}  // namespace Gimbal
}  // namespace Task
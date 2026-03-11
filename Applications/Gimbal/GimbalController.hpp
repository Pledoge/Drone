#include "GimbalCalculator.hpp"
#include "J4310_2EC.hpp"
#include "MF7015.hpp"
#include "PID.hpp"

namespace Task
{
namespace Gimbal
{
class GimbalController
{
   public:
    GimbalController(AbstractFeedbackMotor *_pitch_motor,
                     AbstractFeedbackMotor *_yaw_motor,
                     Core::Control::PID *_yaw_pos_pid,
                     Core::Control::PID *_pitch_pos_pid,
                     Core::Control::PID *_yaw_auto_aim_pid,
                     Core::Control::PID *_pitch_auto_aim_pid)
        : pitch_motor_(_pitch_motor),
          yaw_motor_(_yaw_motor),
          yaw_pos_pid_(_yaw_pos_pid),
          pitch_pos_pid_(_pitch_pos_pid),
          yaw_auto_aim_pid_(_yaw_auto_aim_pid),
          pitch_auto_aim_pid_(_pitch_auto_aim_pid)
    {
    }

    enum class GimbalState : uint8_t
    {
        NORMAL,
        STALL,
        COMPENSATING
    };

    enum class ErrorType : uint8_t
    {
        NO_ERROR,
        PITCH_MOTOR_ERROR,
        YAW_MOTOR_ERROR,
        BOTH_MOTOR_ERROR
    };

    GimbalState getGimbalState() const;

    ErrorType getErrorType()
    {
        if (!yaw_motor_->isConnected() && !pitch_motor_->isConnected())
        {
            error_type_ = ErrorType::BOTH_MOTOR_ERROR;
        }
        else if (!yaw_motor_->isConnected())
        {
            error_type_ = ErrorType::YAW_MOTOR_ERROR;
        }
        else if (!pitch_motor_->isConnected())
        {
            error_type_ = ErrorType::PITCH_MOTOR_ERROR;
        }
        else
        {
            error_type_ = ErrorType::NO_ERROR;
        }
        return error_type_;
    }

    void updateGimbalAttitude();
    void changeTargetPosByCommand(float _delta_yaw, float _delta_pitch);
    void changeTargetPosByTargetCommand(float _yaw_param, float _pitch_param);
    void updateTargetGimbalPos(float _yaw_param, float _pitch_param, bool _target_mode = false);
    void syncYawPosition();
    void syncPitchPosition();

    bool getYawConnected() const { return yaw_motor_->isConnected(); }
    bool getPitchConnected() const { return pitch_motor_->isConnected(); }

    Core::Control::Gimbal::EulerAngles getAngleError() const { return angle_error_; }

    void setYawMotorOutput(bool _disable = false);
    void setPitchMotorOutput(bool _disable = false);
    void setYawAutoAimOutput(bool _disable = false);
    void setPitchAutoAimOutput(bool _disable = false);

    void gimbalStateUpdate();

    void setDisableAll(bool _disable) { disable_all_ = _disable; }
    bool getDisableAll() const { return disable_all_; }

    void setPitchPosition(float _position) { pitch_motor_->setPosition(_position); }
    void setYawPosition(float _position) { yaw_motor_->setPosition(_position); }

    const Core::Control::Gimbal::EulerAngles &getCurrentGimbalEuler() const { return current_gimbal_euler_; }
    const Core::Control::Gimbal::EulerAngles &getCurrentMotorEuler() const { return current_motor_euler_; }
    const Core::Control::Gimbal::EulerAngles &getTargetMotorEuler() const { return target_motor_euler_; }
    void setCompensating(bool _compensating) { is_compensating_ = _compensating; }

   private:
    AbstractFeedbackMotor *pitch_motor_;
    AbstractFeedbackMotor *yaw_motor_;
    Core::Control::PID *yaw_pos_pid_;
    Core::Control::PID *pitch_pos_pid_;
    Core::Control::PID *yaw_auto_aim_pid_;
    Core::Control::PID *pitch_auto_aim_pid_;

    GimbalState gimbal_state_ = GimbalState::STALL;
    ErrorType error_type_     = ErrorType::NO_ERROR;

    Core::Utils::Math::Quaternion<float> current_gimbal_quat_;
    Core::Control::Gimbal::EulerAngles current_gimbal_euler_;
    Core::Control::Gimbal::EulerAngles target_gimbal_euler_;
    Core::Control::Gimbal::EulerAngles current_motor_euler_;
    Core::Control::Gimbal::EulerAngles target_motor_euler_;

    Core::Control::Gimbal::EulerAngles angle_error_;

    float pitch_output_;  // output of the gravity compensation
    float yaw_output_;    // output of the yaw motor

    bool disable_all_ = false;

    bool is_compensating_ = false;
};

Core::Utils::Math::Quaternion<float> getCameraQ();

}  // namespace Gimbal
}  // namespace Task
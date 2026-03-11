#include "Config.hpp"
#include "M2006.hpp"
#include "M3508.hpp"
#include "PID.hpp"
#include "main.h"

namespace Task
{
namespace Shooter
{
class ShooterController
{
   public:
    ShooterController(AbstractFeedbackMotor *_left_shooter_motor,
                      AbstractFeedbackMotor *_right_shooter_motor,
                      AbstractFeedbackMotor *_loader_motor,
                      Core::Control::PID *_left_shooter_pid,
                      Core::Control::PID *_right_shooter_pid,
                      Core::Control::PID *_loader_pid)
        : left_shooter_motor_(_left_shooter_motor),
          right_shooter_motor_(_right_shooter_motor),
          loader_motor_(_loader_motor),
          left_shooter_pid_(_left_shooter_pid),
          right_shooter_pid_(_right_shooter_pid),
          loader_pid_(_loader_pid){};

    enum class ShooterState : uint8_t
    {
        IDLE,
        SHOOTING,
        LOADING,  // When loading, it must shooting
        STUCK
    };

    enum class ErrorType : uint8_t
    {
        NO_ERROR,
        LEFT_SHOOTER_MOTOR_ERROR,
        RIGHT_SHOOTER_MOTOR_ERROR,
        LOADER_MOTOR_ERROR,
        MULTIPLE_MOTOR_ERROR
    };

    void updateShooterState(bool _shoot_command, bool _load_command);

    ShooterState getShooterState() const { return shooter_state_; }
    ErrorType getErrorType()
    {
        if (!left_shooter_motor_->isConnected() && !right_shooter_motor_->isConnected() && !loader_motor_->isConnected())
        {
            error_type_ = ErrorType::MULTIPLE_MOTOR_ERROR;
        }
        else if (!left_shooter_motor_->isConnected())
        {
            error_type_ = ErrorType::LEFT_SHOOTER_MOTOR_ERROR;
        }
        else if (!right_shooter_motor_->isConnected())
        {
            error_type_ = ErrorType::RIGHT_SHOOTER_MOTOR_ERROR;
        }
        else if (!loader_motor_->isConnected())
        {
            error_type_ = ErrorType::LOADER_MOTOR_ERROR;
        }
        else
        {
            error_type_ = ErrorType::NO_ERROR;
        }
        return error_type_;
    }

    bool getShooterConnected() const
    {
        return left_shooter_motor_->isConnected() && right_shooter_motor_->isConnected() && loader_motor_->isConnected();
    }

    bool getLoaderConnected() const { return loader_motor_->isConnected(); }

    void triggerShooter();
    void triggerLoader();
    void disableShooter();
    void stopLoader();
    void disableLoader();

    void stuckChecker();
    void stuckHandler();

    void setDisableAll(bool _disable) { disable_all_ = _disable; }
    bool getDisableAll() const { return disable_all_; }

    bool checkShootingState();

    void updateAccRadio(bool _is_shooting);

   private:
    AbstractFeedbackMotor *left_shooter_motor_;
    AbstractFeedbackMotor *right_shooter_motor_;
    AbstractFeedbackMotor *loader_motor_;

    Core::Control::PID *left_shooter_pid_;
    Core::Control::PID *right_shooter_pid_;
    Core::Control::PID *loader_pid_;

    ShooterState shooter_state_ = ShooterState::IDLE;
    ErrorType error_type_       = ErrorType::NO_ERROR;

    float acc_radio_ = 0.0f;

    float last_stuck_angle_;
    bool is_stuck_ = false;

    bool disable_all_ = false;
};
}  // namespace Shooter
}  // namespace Task
#include "GimbalTask.hpp"

#include "CommandTask.hpp"
#include "Compensation.hpp"
#include "Config.hpp"
#include "GM6020.hpp"
#include "J4310_2EC.hpp"
#include "MF7015.hpp"
#include "cvTest.hpp"
namespace Task
{
namespace Gimbal
{
Core::Drivers::Motors::GM6020 yaw_motor(3, 1, Core::Drivers::Motors::GM6020::ControlMode::VOLTAGE, true);
Core::Drivers::Motors::J4310_2EC<AbstractFeedbackMotor::CommMode::OneToOne> pitch_motor(
    0x309, 1, 1, Core::Drivers::Motors::DMMotorComm<AbstractFeedbackMotor::CommMode::OneToOne, false>::EMotorMode::eModeMIT, false);

Core::Control::PID::Param yaw_pos_pid_param(250000.0f, 0.0f, 6000.0f, 10000, 10000, 0, 0.0f, 0.1f);
Core::Control::PID::Param yaw_pos_pid_param_auto_aim(200000.0f, 0.0f, 12000.0f, 100000, 100000, 0, 0.0f, 0.05f);
// Core::Control::PID::Param yaw_pos_pid_param(0);
Core::Control::PID yaw_pos_pid(yaw_pos_pid_param);
Core::Control::PID yaw_auto_aim_pid(yaw_pos_pid_param_auto_aim);

Core::Control::PID::Param pitch_pos_pid_param(35, 0.0f, 6, 10000, 10000, 0);
Core::Control::PID::Param pitch_pos_pid_param_auto_aim(25, 0, 20, 10000, 10000, 0, 0, 0.05f);
// Core::Control::PID::Param pitch_pos_pid_param(0);
Core::Control::PID pitch_pos_pid(pitch_pos_pid_param);
Core::Control::PID pitch_auto_aim_pid(pitch_pos_pid_param_auto_aim);

GimbalController gimbal_controller(&pitch_motor, &yaw_motor, &yaw_pos_pid, &pitch_pos_pid, &yaw_auto_aim_pid, &pitch_auto_aim_pid);

const CommandTask::GimbalCommand &gimbal_command_copy = Task::CommandTask::getGimbalCommand();

static GimbalController::GimbalState gimbal_state = GimbalController::GimbalState::STALL;
static GimbalController::ErrorType gimbal_error   = GimbalController::ErrorType::NO_ERROR;

// Compensation Range
Core::Control::PID::Param pitch_compensation_speed_pid_param(0.0f, 0, 0, 10000, 10000, 0);
Core::Control::PID pitch_compensation_speed_pid(pitch_compensation_speed_pid_param);
// End

static volatile bool ros_yaw_pid_test_flag = false;

void gimbalTask(void *pvPara)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    //
    gimbal_controller.gimbalStateUpdate();
    yaw_motor.enable();
    pitch_motor.enable();
    vTaskDelay(3000);
    yaw_motor.setPosition(Core::Control::Gimbal::clampGimbalError(yaw_motor.getPositionFeedback() + MIN_YAW_MOTOR_POSITION + MIN_YAW_ANGLE));
    pitch_motor.setPosition(Core::Control::Gimbal::clampGimbalError(pitch_motor.getPositionFeedback() - FLAT_PITCH_MOTOR_POSITION));

    gimbal_controller.updateGimbalAttitude();
    vTaskDelay(10);
    gimbal_controller.syncYawPosition();
    gimbal_controller.syncPitchPosition();
    while (true)
    {
        if (gimbal_command_copy.controlMode != CommandTask::GimbalCommand::ControlMode::RC)
        {
            gimbal_controller.setDisableAll(true);
        }
        else
        {
            gimbal_controller.setDisableAll(false);
        }

        gimbal_state = gimbal_controller.getGimbalState();
        gimbal_error = gimbal_controller.getErrorType();

        switch (gimbal_state)
        {
        case GimbalController::GimbalState::NORMAL:
        {
            if (!gimbal_command_copy.getAutoShootMode())
            {
                gimbal_controller.updateTargetGimbalPos(gimbal_command_copy.getDeltaYaw(), gimbal_command_copy.getDeltaPitch(), false);
                gimbal_controller.setPitchMotorOutput();
                gimbal_controller.setYawMotorOutput();
            }
            else
            {
                gimbal_controller.updateTargetGimbalPos(gimbal_command_copy.getTargetYaw(), gimbal_command_copy.getTargetPitch(), true);
                gimbal_controller.setPitchAutoAimOutput();
                gimbal_controller.setYawAutoAimOutput();
            }
            break;
        }

        case GimbalController::GimbalState::STALL:
        {
            if (gimbal_controller.getDisableAll())
            {
                gimbal_controller.updateTargetGimbalPos(0, 0);
                gimbal_controller.setPitchMotorOutput(true);
                gimbal_controller.setYawMotorOutput(true);
                gimbal_controller.setYawAutoAimOutput(true);
                gimbal_controller.setPitchAutoAimOutput(true);
                gimbal_controller.syncPitchPosition();
                gimbal_controller.syncYawPosition();
                break;
            }
            gimbal_controller.updateTargetGimbalPos(0, 0);
            gimbal_controller.setPitchMotorOutput();
            gimbal_controller.setYawMotorOutput();
            gimbal_controller.setYawAutoAimOutput();
            gimbal_controller.setPitchAutoAimOutput();
            gimbal_controller.syncPitchPosition();
            gimbal_controller.syncYawPosition();
            break;
        }
        case GimbalController::GimbalState::COMPENSATING:
        {
#define ONE_STEP_ANGLE -0.01f
            static bool finish_one_step = false;
            static bool finish_all      = false;
            static uint32_t start_time  = xTaskGetTickCount();
            if (!finish_all)
            {
                uint32_t current_time = xTaskGetTickCount();

                if (current_time - start_time > pdMS_TO_TICKS(5000))
                {
                    finish_one_step =
                        Compensation::updatePitchCompensationData(gimbal_controller.getCurrentGimbalEuler().pitch, pitch_motor.getCurrentFeedback());
                }
                if (finish_one_step)
                {
                    gimbal_controller.changeTargetPosByCommand(0, ONE_STEP_ANGLE);
                    start_time      = xTaskGetTickCount();
                    finish_one_step = false;
                }
                if (gimbal_controller.getTargetMotorEuler().pitch + ONE_STEP_ANGLE <= MIN_PITCH_ANGLE)
                {
                    finish_all = true;
                    gimbal_controller.setCompensating(false);
                }
            }
            gimbal_controller.updateTargetGimbalPos(0, 0);
            gimbal_controller.setPitchMotorOutput();
            gimbal_controller.setYawMotorOutput();
            break;
        }
        default:
        {
            break;
        }
        }
        gimbal_controller.gimbalStateUpdate();
        vTaskDelayUntil(&last_wake_time, 1);
    }
}

StackType_t uxGimbalTaskStack[512];
StaticTask_t xGimbalTaskTCB;

Core::Drivers::Motors::J4310_2EC<AbstractFeedbackMotor::CommMode::OneToOne> &getPitchMotor() { return pitch_motor; }

static bool initialized = false;

void init()
{
    configASSERT(!initialized);
    initialized = true;
    xTaskCreateStatic(gimbalTask, "GimbalTask", 512, NULL, 2, uxGimbalTaskStack, &xGimbalTaskTCB);
}

const volatile Core::Control::Gimbal::EulerAngles &getCurrentGimbalEuler() { return gimbal_controller.getCurrentGimbalEuler(); }
}  // namespace Gimbal
}  // namespace Task
#include "ShooterTask.hpp"

#include "CommandTask.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace Task
{
namespace Shooter
{
//
Core::Drivers::Motors::M3508 left_shooter_motor(1, 1, 1, 1, false);
Core::Drivers::Motors::M3508 right_shooter_motor(2, 1, 1, 1, true);
Core::Drivers::Motors::M2006 loader_motor(4, 1);

Core::Control::PID::Param left_shooter_pid_param(0.01f, 0.0f);
Core::Control::PID left_shooter_pid(left_shooter_pid_param);

Core::Control::PID::Param right_shooter_pid_param(0.01f, 0.0f);
Core::Control::PID right_shooter_pid(right_shooter_pid_param);

Core::Control::PID::Param loader_pid_param(150, 0, 3.5);
Core::Control::PID loader_pid(loader_pid_param);

ShooterController shooter_controller(&left_shooter_motor, &right_shooter_motor, &loader_motor, &left_shooter_pid, &right_shooter_pid, &loader_pid);
CommandTask::ShooterCommand const &shooter_command_copy = Task::CommandTask::getShooterCommand();

static ShooterController::ShooterState shooter_state = ShooterController::ShooterState::IDLE;
static ShooterController::ErrorType shooter_error    = ShooterController::ErrorType::NO_ERROR;

static volatile float output_left = 0.0f;
void shooterTask(void *pvPara)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    left_shooter_motor.enable();
    right_shooter_motor.enable();
    loader_motor.enable();
    shooter_controller.updateShooterState(shooter_command_copy.getShootCommand(), shooter_command_copy.getLoadCommand());
    while (true)
    {
        shooter_state = shooter_controller.getShooterState();
        shooter_error = shooter_controller.getErrorType();
        shooter_controller.updateAccRadio(shooter_command_copy.getShootCommand());
        shooter_controller.setDisableAll(CommandTask::Command::controlMode != CommandTask::Command::ControlMode::RC);
        switch (shooter_state)
        {
        case ShooterController::ShooterState::IDLE:
        {
            shooter_controller.disableShooter();
            shooter_controller.stopLoader();
            break;
        }
        case ShooterController::ShooterState::SHOOTING:
        {
            shooter_controller.triggerShooter();
            shooter_controller.stopLoader();
            break;
        }
        case ShooterController::ShooterState::LOADING:
        {
            shooter_controller.triggerShooter();
            shooter_controller.triggerLoader();
            shooter_controller.stuckChecker();
            break;
        }
        case ShooterController::ShooterState::STUCK:
        {
            shooter_controller.triggerShooter();
            shooter_controller.stuckHandler();
            break;
        }
        }
        shooter_controller.updateShooterState(shooter_command_copy.getShootCommand(), shooter_command_copy.getLoadCommand());
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}

StackType_t uxShooterTaskStack[256];
StaticTask_t xShooterTaskTCB;

static bool initialized = false;
void init()
{
    configASSERT(!initialized);
    initialized = true;
    xTaskCreateStatic(shooterTask, "ShooterTask", 256, NULL, 2, uxShooterTaskStack, &xShooterTaskTCB);
}

}  // namespace Shooter
}  // namespace Task
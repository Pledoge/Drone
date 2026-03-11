#include "CommandTask.hpp"

#include <cmath>

#include "Config.hpp"
#include "DJIMotor.hpp"
#include "DMMotor.hpp"
#include "DR16.hpp"
#include "FreeRTOS.h"
#include "GimbalTask.hpp"
#include "KeyboardManager.hpp"
#include "LKMotor.hpp"
#include "RosTask.hpp"
#include "VTMComm.hpp"
#include "task.h"

namespace Task
{
namespace CommandTask
{

const volatile Core::Drivers::DR16::RcData &rc_data_copy                            = Core::Drivers::DR16::getRcData();
const volatile Core::Drivers::VTMComm::VTMData &vtm_data_copy                       = Core::Drivers::VTMComm::getVTMData();
const volatile Core::Communication::RosComm::GimbalCommand &ros_gimbal_command_copy = RosTask::getRosGimbal();
const volatile Core::Control::Gimbal::EulerAngles &current_gimbal_euler_copy        = Gimbal::getCurrentGimbalEuler();

uint8_t Command::command_count_                  = 0;
Command *Command::all_commands_[MAX_COMMAND_NUM] = {nullptr};
Command::ControlMode Command::controlMode        = Command::ControlMode::DISABLED;

GimbalCommand gimbal_command;
ShooterCommand shooter_command;

void Command::updateAllCommands()
{
    for (uint8_t i = 0; i < command_count_; ++i)
    {
        all_commands_[i]->update();
    }
}

void Command::updateControlStatus()
{
    using namespace Core::Drivers::DR16;

    if (!rc_data_copy.is_connected || (rc_data_copy.rc.s1 == 1))
    {
        controlMode = ControlMode::DISABLED;
        return;
    }
    if (rc_data_copy.rc.s1 == 3)
    {
        controlMode = ControlMode::RC;
    }
    else if (rc_data_copy.rc.s1 == 2)
    {
        controlMode = ControlMode::VTM_RC;
    }
    else
    {
        controlMode = ControlMode::DISABLED;
    }
}

void GimbalCommand::update()
{
    using namespace Core::Drivers::DR16;
    static auto handleStickDeadzone = [](int16_t value) -> int { return fabs(value) > 20 ? value : 0; };
    if (Command::controlMode == ControlMode::RC)
    {
        // Assume channel 1 is yaw and channel 2 is pitch
        float yaw_input   = static_cast<float>(rc_data_copy.rc.ch0) - 1024.0f;  // Normalize to [-1, 1]
        float pitch_input = static_cast<float>(rc_data_copy.rc.ch1) - 1024.0f;  // Normalize to [-1, 1]
        if (!shooter_command.getAutoAimMode())
        {
            yaw_input   = static_cast<float>(rc_data_copy.rc.ch0) - 1024.0f;
            pitch_input = static_cast<float>(rc_data_copy.rc.ch1) - 1024.0f;
            // Scale inputs to desired range, e.g., [-30, 30] degrees
            delta_yaw_   = -RC_YAW_SENSITIVITY * handleStickDeadzone(yaw_input);
            delta_pitch_ = -RC_PITCH_SENSITIVITY * handleStickDeadzone(pitch_input);
        }
        else
        {
            if (ros_gimbal_command_copy.shoot_mode == 0)
            {
                yaw_input   = static_cast<float>(rc_data_copy.rc.ch0) - 1024.0f;
                pitch_input = static_cast<float>(rc_data_copy.rc.ch1) - 1024.0f;
                // Scale inputs to desired range, e.g., [-30, 30] degrees
                delta_yaw_   = -RC_YAW_SENSITIVITY * handleStickDeadzone(yaw_input);
                delta_pitch_ = -RC_PITCH_SENSITIVITY * handleStickDeadzone(pitch_input);
            }
            else
            {
                target_yaw_   = ros_gimbal_command_copy.target_yaw;
                target_pitch_ = ros_gimbal_command_copy.target_pitch;
            }
        }
    }
    else if (Command::controlMode == ControlMode::VTM_RC)
    {
        // Assume channel 1 is yaw and channel 2 is pitch, but with different scaling
        float yaw_input   = static_cast<float>(vtm_data_copy.rc.ch0) - 1024.0f;  // Normalize to [-1, 1]
        float pitch_input = static_cast<float>(vtm_data_copy.rc.ch1) - 1024.0f;  // Normalize to [-1, 1]
        // Scale inputs to desired range, e.g., [-30, 30] degrees
        delta_yaw_   = -RC_YAW_SENSITIVITY * handleStickDeadzone(yaw_input);
        delta_pitch_ = -RC_PITCH_SENSITIVITY * handleStickDeadzone(pitch_input);
    }
    else if (Command::controlMode == ControlMode::KEYBOARD_RC)
    {
        delta_yaw_   = -rc_data_copy.mouse.x * 3.0f / 132000.0f;
        delta_pitch_ = -rc_data_copy.mouse.y * 3.5f * 0.6f / 132000.0f;
    }
    else if (Command::controlMode == ControlMode::KEYBOARD_VTM)
    {
        delta_yaw_   = -vtm_data_copy.mouse.x * 3.0f / 132000.0f;
        delta_pitch_ = -vtm_data_copy.mouse.y * 3.5f * 0.6f / 132000.0f;
    }
    else
    {
        delta_yaw_   = 0.0f;
        delta_pitch_ = 0.0f;
    }
}

bool GimbalCommand::getAutoAimMode() const { return shooter_command.getAutoAimMode(); }
bool GimbalCommand::getAutoShootMode() const { return shooter_command.getAutoAimMode() && ros_gimbal_command_copy.shoot_mode != 0; }

void ShooterCommand::update()
{
    using namespace Core::Drivers::DR16;

    if (Command::controlMode == ControlMode::RC)
    {
        if (rc_data_copy.rc.ch4 > 1500)
        {
            auto_aiming_ = true;
        }
        else if (rc_data_copy.rc.ch4 < 500)
        {
            auto_aiming_ = false;
        }
        shoot_command  = (rc_data_copy.rc.s2 == 3) || (rc_data_copy.rc.s2 == 2);  // Assume s2 in position 3 or 2 means shoot
        load_command   = (rc_data_copy.rc.s2 == 2) || (auto_aiming_ && rc_data_copy.rc.s2 == 2 && ros_gimbal_command_copy.shoot_mode == 2);
        auto_shooting_ = auto_aiming_ && shoot_command && (ros_gimbal_command_copy.shoot_mode == 2 || ros_gimbal_command_copy.shoot_mode == 1);
    }
    else
    {
        shoot_command  = false;
        load_command   = false;
        auto_shooting_ = false;
        auto_aiming_   = false;
    }
}

void commandTask(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    auto &pitch_motor_copy    = Gimbal::getPitchMotor();
    while (1)
    {
        Command::updateControlStatus();
        Command::updateAllCommands();
        // Core::Drivers::Motors::DJIMotor::transmit(0, 0);
        Core::Drivers::Motors::DJIMotor::transmit(0, 1);
        Core::Drivers::Motors::DJIMotor::transmit(1, 1);
        Core::Drivers::Motors::DJIMotor::transmit(1, 2);
        Core::Drivers::Motors::DJIMotor::transmit(0, 2);
        pitch_motor_copy.transmit();
        // Core::Drivers::Motors::DMMotor::transmit(1);
        vTaskDelayUntil(&last_wake_time, 1);
    }
}
using namespace Core::Drivers;
class KeyInput
{
   public:
    Keyboard::Key key_f{Keyboard::Key::Code::F};
    Keyboard::Key key_right{Keyboard::Key::Code::RIGHT_BUTTON};
    Keyboard::Key key_w{Keyboard::Key::Code::W};
    Keyboard::Key key_s{Keyboard::Key::Code::S};
    Keyboard::Key key_a{Keyboard::Key::Code::A};
    // Keyboard::Key key_d{Keyboard::Key::Code::D};
    Keyboard::Key key_r{Keyboard::Key::Code::R};
    Keyboard::Key key_left{Keyboard::Key::Code::LEFT_BUTTON};
    Keyboard::Key key_z{Keyboard::Key::Code::Z};
    Keyboard::Key key_x{Keyboard::Key::Code::X};
    Keyboard::Key key_q{Keyboard::Key::Code::Q};             // Re-initialize pitch and yaw
    Keyboard::Key key_g{Keyboard::Key::Code::G};             // Show armor
    Keyboard::Key key_b{Keyboard::Key::Code::B};             // Menu key
    Keyboard::Key key_e{Keyboard::Key::Code::E};             // Change armor show status
    Keyboard::Key key_ctrl{Keyboard::Key::Code::LEFT_CTRL};  // Control key
    void init()
    {
        Keyboard::KeyManager::manager.init();
        Keyboard::KeyManager::manager.setBelievedDataSource(Keyboard::KeyManager::DataSource::DR16);

        key_w.registerCallback<Keyboard::Key::CallbackRegisterType::PRESS_TO_RELEASE>([]() { gimbal_command.addAutoAimOffset(-0.005f, 0); });
        key_s.registerCallback<Keyboard::Key::CallbackRegisterType::PRESS_TO_RELEASE>([]() { gimbal_command.addAutoAimOffset(0.005f, 0); });
        key_a.registerCallback<Keyboard::Key::CallbackRegisterType::PRESS_TO_RELEASE>([]() { gimbal_command.addAutoAimOffset(0, 0.005f); });
        // key_d.registerCallback<Keyboard::Key::CallbackRegisterType::PRESS_TO_RELEASE>([]() { gimbal_command.addAutoAimOffset(0, -0.005f); });
        // key_q.registerCallback<Keyboard::Key::CallbackRegisterType::RELEASE_TO_PRESS>([]() { shooterCommand.switchOneConsistentBullet(); });
        // key_f.registerCallback<Keyboard::Key::CallbackRegisterType::RELEASE_TO_PRESS>([]() { shooterCommand.changeShooterState(); });
        key_g.registerCallback<Keyboard::Key::CallbackRegisterType::RELEASE_TO_PRESS>([]() { gimbal_command.resetAutoAimOffset(); });
        key_b.registerCallback<Keyboard::Key::CallbackRegisterType::LONG_PRESS>([]() { HAL_NVIC_SystemReset(); });
        // key_e.registerCallback<Keyboard::Key::CallbackRegisterType::RELEASE_TO_PRESS>([]() { shooterCommand.switchAutoMode(); });
        // key_left.registerCallback<Keyboard::Key::CallbackRegisterType::PRESS_TO_RELEASE>([]() { shooterCommand.enableFrictionWheels(); });
        // key_ctrl.registerCallback<Keyboard::Key::CallbackRegisterType::LONG_PRESS>([]() { Command::switchDisableAllCommands(); });
    }
} key_input;

StackType_t uxCommandTaskStack[256];
StaticTask_t xCommandTaskTCB;

void initCommandTask()
{
    key_input.init();
    xTaskCreateStatic(commandTask, "CommandTask", 256, nullptr, 10, uxCommandTaskStack, &xCommandTaskTCB);
}

const GimbalCommand &getGimbalCommand() { return gimbal_command; }
const ShooterCommand &getShooterCommand() { return shooter_command; }

}  // namespace CommandTask
}  // namespace Task
#include "main.h"

namespace Task
{
namespace CommandTask
{
#define MAX_COMMAND_NUM 5

class Command
{
   public:
    Command() { all_commands_[command_count_++] = this; }
    virtual void update() = 0;

    static void updateControlStatus();

    static void updateAllCommands();

    enum class ControlMode
    {
        DISABLED,
        RC,
        VTM_RC,
        KEYBOARD_RC,
        KEYBOARD_VTM,
    };
    static ControlMode controlMode;

   private:
    static uint8_t command_count_;
    static Command *all_commands_[MAX_COMMAND_NUM];
};

class GimbalCommand : public Command
{
   public:
    GimbalCommand() {}
    void update() override;

    float getDeltaYaw() const { return delta_yaw_; }
    float getDeltaPitch() const { return delta_pitch_; }
    float getTargetYaw() const { return target_yaw_; }
    float getTargetPitch() const { return target_pitch_; }
    bool getAutoAimMode() const;
    bool getAutoShootMode() const;
    void addAutoAimOffset(float pitch_offset, float yaw_offset)
    {
        yaw_autoaim_offset_   = yaw_offset;
        pitch_autoaim_offset_ = pitch_offset;
    }
    void resetAutoAimOffset()
    {
        yaw_autoaim_offset_   = 0.0f;
        pitch_autoaim_offset_ = 0.0f;
    }

   private:
    float delta_yaw_   = 0.0f;
    float delta_pitch_ = 0.0f;

    float target_yaw_   = 0.0f;
    float target_pitch_ = 0.0f;

    float yaw_autoaim_offset_   = 0.0f;
    float pitch_autoaim_offset_ = 0.0f;
};

class ShooterCommand : public Command
{
   public:
    ShooterCommand() {}
    void update() override;

    bool getShootCommand() const { return shoot_command; }
    bool getLoadCommand() const { return load_command; }
    void setAutoMode(bool auto_mode) { auto_aiming_ = auto_mode; }
    bool getAutoAimMode() const { return auto_aiming_; }
    void setAutoShooting(bool auto_shooting) { auto_shooting_ = auto_shooting; }
    bool getAutoShooting() const { return auto_shooting_; }

   private:
    bool shoot_command  = false;
    bool load_command   = false;
    bool auto_aiming_   = false;
    bool auto_shooting_ = false;
};

const GimbalCommand &getGimbalCommand();
const ShooterCommand &getShooterCommand();

void initCommandTask();

}  // namespace CommandTask
}  // namespace Task
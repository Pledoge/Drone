#include "RefereeTask.hpp"

#include "KeyboardManager.hpp"
#include "RefereeClientUIItem.hpp"

namespace Task
{
namespace RefereeCommTask
{
using namespace Core::Communication::RefereeSystem;

#define SIGN(x) ((x >= 0) ? 1 : 0)

Core::Drivers::Keyboard::Key key_d(Core::Drivers::Keyboard::Key::Code::D);

void correctId(RefereeRobotStatusMessageData data) { senderRobotID = (uint16_t)data.robot_id; }

RefereeSystemMessageReceive<RefereeRobotStatusMessageData> refereeRobotStatusMessageReceiver;
// RefereeSystemMessageReceive<RefereeAirSupportMessageData> refereeAirSupportMessageReceiver((uint16_t)RefereeMessageID::AIR_SUPPORT_DATA_ID);
RefereeSystemMessageReceive<RefereeShootingMessageData> refereeShootingMessageReceiver((uint16_t)RefereeMessageID::SHOOT_DATA_ID);

RefereeSystemMessageReceive<RefereeProjectileAllowanceMessageData> refereeProjectileAllowanceMessageReceiver(
    (uint16_t)RefereeMessageID::PROJECTILE_ALLOWANCE_ID);

RefereeSystemMessageReceive<RefereeRadarInfoMessageData> referee_radar_info_message_receiver((uint16_t)RefereeMessageID::RADAR_INFO_ID);

const RefereeRobotStatusMessageData &robotStatus                 = refereeRobotStatusMessageReceiver.getData();
const RefereeShootingMessageData &shooting                       = refereeShootingMessageReceiver.getData();
const RefereeProjectileAllowanceMessageData &projectileAllowance = refereeProjectileAllowanceMessageReceiver.getData();

const RefereeRadarInfoMessageData &radarInfo = referee_radar_info_message_receiver.getData();

RefereeData referee_data;

const RefereeData &getRefereeData() { return referee_data; }

ClientUIIcon autoaim_status(IconType::INTEGER, 0, (uint32_t)Color::PINK, 40, 0, 4, 936, 280, 0, 0, 0, 10, 2000);

ClientUIText shoot_mode(IconType::TEXT, 0, (uint32_t)Color::PINK, 20, 6, 2, 1600, 480, 0, 0, 0, 10, 2000);
ClientUIText auto_mode(IconType::TEXT, 0, (uint32_t)Color::PINK, 20, 4, 2, 1618, 429, 0, 0, 0, 10, 2000);
ClientUIIcon pitch_offset(IconType::FLOAT, 0, (uint32_t)Color::TEAM_COLOR, 20, 0, 2, 128, 650, 57, 12, 0, 1000);
ClientUIIcon yaw_offset(IconType::FLOAT, 0, (uint32_t)Color::TEAM_COLOR, 20, 0, 2, 128, 697, 57, 12, 0, 2000);
ClientUIIcon radar_double_vulnerable_times(IconType::INTEGER, 0, (uint32_t)Color::TEAM_COLOR, 20, 0, 2, 1602, 375, 57, 12, 0, 5, 2000);

ClientUIText pitch_motifier(IconType::TEXT, 0, (uint32_t)Color::TEAM_COLOR, 20, 1, 2, 85, 749, 0, 0, 0, 1);
ClientUIText yaw_motifier(IconType::TEXT, 0, (uint32_t)Color::TEAM_COLOR, 20, 1, 2, 82, 693, 0, 0, 0, 1);
ClientUIText radar_notifier(IconType::TEXT, 0, (uint32_t)Color::TEAM_COLOR, 20, 1, 2, 1559, 373, 0, 0, 0, 1);

ClientUIIcon Autoaim_offset_pitch(IconType::INTEGER, 0, (uint32_t)Color::TEAM_COLOR, 20, 0, 2, 283, 747, 57, 12, 0, 10);
ClientUIIcon Autoaim_offset_yaw(IconType::INTEGER, 0, (uint32_t)Color::TEAM_COLOR, 20, 0, 2, 284, 699, 57, 12, 0, 10);
ClientUIIcon auto_aim_target(IconType::CIRCLE, 0, (uint32_t)Color::TEAM_COLOR, 0, 0, 5, 640, 483, 28, 0, 0, 10);

ClientUIIcon minus_flag1(IconType::LINE, 0, (uint32_t)Color::TEAM_COLOR, 0, 0, 1, 263, 737, 0, 276, 737, 5);
ClientUIIcon minus_flag2(IconType::LINE, 0, (uint32_t)Color::TEAM_COLOR, 0, 0, 1, 263, 688, 0, 276, 688, 5);

ClientUIIcon target_armor_pose(IconType::CIRCLE, 0, (uint32_t)Color::TEAM_COLOR, 0, 0, 5, 0, 0, 28, 0, 0, 30);

ClientUIIcon auto_aim_range(IconType::RECTANGLE, 0, (uint32_t)Color::TEAM_COLOR, 0, 0, 1, 790, 341, 0, 1146, 728, 1);

static bool is_show = true;

void refereeTask(void *pvPara)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    shoot_mode.init();
    shoot_mode.add();
    autoaim_status.init();
    autoaim_status.add();
    auto_mode.init();
    auto_mode.add();
    pitch_offset.init();
    pitch_offset.add();
    yaw_offset.init();
    yaw_offset.add();
    radar_double_vulnerable_times.init();
    radar_double_vulnerable_times.add();
    auto_aim_range.init();
    auto_aim_range.add();
    pitch_motifier.init();
    pitch_motifier.add();
    yaw_motifier.init();
    yaw_motifier.add();

    target_armor_pose.init();
    target_armor_pose.add();

    auto_aim_target.init();
    Autoaim_offset_pitch.init();
    Autoaim_offset_yaw.init();
    Autoaim_offset_pitch.add();
    Autoaim_offset_yaw.add();
    minus_flag1.init();
    minus_flag2.init();

    minus_flag1.add();
    minus_flag2.add();
    static uint32_t test_value = 0;
    while (true)
    {
        correctId(robotStatus);
        referee_data.robotID = (uint8_t)robotStatus.robot_id;
        // Get robot data
        referee_data.robotColor       = (Task::RefereeCommTask::RefereeData::Color)SIGN((int8_t)robotStatus.robot_id - 100);
        referee_data.shootingFreq     = shooting.launching_freq;
        referee_data.lastInitialSpeed = shooting.bullet_initial_speed;
        referee_data.remainingAmmo    = projectileAllowance.projectile_allowance_17mm;
        referee_data.radar_times      = radarInfo.double_vulnerable_times;

        shoot_mode.modify();
        autoaim_status.modify();
        auto_mode.modify();
        Autoaim_offset_pitch.modifyAttribute(Attributes::INTEGER_VALUE, test_value++);
        Autoaim_offset_pitch.modify();
        Autoaim_offset_yaw.modifyAttribute(Attributes::INTEGER_VALUE, test_value++);
        Autoaim_offset_yaw.modify();
        pitch_motifier.modify();
        yaw_motifier.modify();
        minus_flag1.modify();
        minus_flag2.modify();
        auto_aim_range.modify();
        pitch_offset.modify();
        yaw_offset.modify();
        target_armor_pose.modify();
        radar_double_vulnerable_times.modify();

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}

StackType_t uxRefereeTaskStack[256];
StaticTask_t xRefereeTaskTCB;

static bool initialized = false;

void init()
{
    configASSERT(!initialized);
    initialized = true;
    Core::Drivers::Keyboard::KeyManager::manager.init();
    Core::Drivers::Keyboard::KeyManager::manager.setBelievedDataSource(Core::Drivers::Keyboard::KeyManager::DataSource::VTM);
    Core::Drivers::Keyboard::KeyManager::manager.registerKeyCallback<Core::Drivers::Keyboard::Key::CallbackRegisterType::PRESS_TO_RELEASE>(
        Core::Drivers::Keyboard::Key::Code::D,
        []()
        {
            if (is_show)
            {
                auto_aim_range.del();
                is_show = false;
            }
            else
            {
                auto_aim_range.add();
                is_show = true;
            }
        });
    xTaskCreateStatic(refereeTask, "RefereeTask", 256, NULL, 11, uxRefereeTaskStack, &xRefereeTaskTCB);
}
}  // namespace RefereeCommTask
}  // namespace Task
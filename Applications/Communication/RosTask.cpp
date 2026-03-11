#include "RosTask.hpp"

#include "Config.hpp"
#include "GimbalController.hpp"
#include "RefereeSystemComm.hpp"
#include "RefereeTask.hpp"
#include "usart.h"

namespace Task
{
namespace RosTask
{

using namespace Core::Communication;
using namespace Core::Communication::RosComm;
using namespace Core::Communication::RefereeSystem;

// Transmit
GimbalMsg gimbalStatus;
static uint8_t rosTxBuff[sizeof(GimbalMsg)] __attribute((used, section(".D1")));

// Receive
GimbalCommand rosGimbalCommand;
RefereeSystemMessageReceive<RefereeRobotStatusMessageData> refereeRobotStatusMessageReceiver((uint16_t)RefereeMessageID::ROBOT_STATUS_ID);
const RefereeRobotStatusMessageData &robotStatus = refereeRobotStatusMessageReceiver.getData();
FrameHeader GimbalStatusHandle                   = {START_BYTE, sizeof(GimbalMsg), RosComm::CommunicationType::TWOCRC_GIMBAL_MSG};

const RefereeCommTask::RefereeData &refereeData_copy = RefereeCommTask::getRefereeData();
void rosTask(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        gimbalStatus.bullet_speed                    = refereeData_copy.lastInitialSpeed;
        gimbalStatus.cur_cv_mode                     = 0;  // TODO: set CV mode
        gimbalStatus.target_color                    = static_cast<uint8_t>(refereeData_copy.robotColor);
        gimbalStatus.target_locked                   = 0;  // TODO: set target locked status
        Core::Utils::Math::Quaternion<float> cameraQ = Task::Gimbal::getCameraQ();
        gimbalStatus.q[0]                            = cameraQ[0];
        gimbalStatus.q[1]                            = cameraQ[1];
        gimbalStatus.q[2]                            = cameraQ[2];
        gimbalStatus.q[3]                            = cameraQ[3];
        gimbalStatus.shoot_delay                     = 1.0f / LOADER_FREQUENCY;
        memcpy(rosTxBuff, &gimbalStatus, sizeof(GimbalMsg));
        Core::Communication::RosComm::RosManager::managers[0].transmit(GimbalStatusHandle, rosTxBuff);

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1));
    }
}

StackType_t uxRosTaskStack[128];
StaticTask_t xRosTaskTCB;

static bool initialized = false;

// Receive part NUC->PCB
void decodeROSGimbal(uint8_t *data, uint16_t len, UART_HandleTypeDef *handle) { memcpy(&rosGimbalCommand, data, len); }

void init()
{
    configASSERT(!initialized);
    initialized = true;
    Core::Communication::RosComm::RosManager::managers[0].init(&huart2);
    Core::Communication::RosComm::RosManager::managers[0].registerFrameCallback(CommunicationType::TWOCRC_GIMBAL_CMD, decodeROSGimbal);
    xTaskCreateStatic(rosTask, "RosTask", 128, NULL, 10, uxRosTaskStack, &xRosTaskTCB);
}

const volatile Core::Communication::RosComm::GimbalCommand &getRosGimbal() { return rosGimbalCommand; }

}  // namespace RosTask
}  // namespace Task
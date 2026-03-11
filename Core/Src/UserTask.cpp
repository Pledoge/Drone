/**
 * @file UserTask.cpp
 * @author JIANG Yicheng  RM2023 (EthenJ@outlook.sg)
 * @brief Create user tasks with cpp support
 * @version 0.1
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 */

#include "CANManager.hpp"
#include "CommandTask.hpp"
#include "DJIMotor.hpp"
#include "DR16.hpp"
#include "FreeRTOS.h"
#include "GimbalTask.hpp"
#include "IMU.hpp"
#include "J4310_2EC.hpp"
#include "MF7015.hpp"
#include "PowerManager.hpp"
#include "RefereeSystemComm.hpp"
#include "RefereeSystemManager.hpp"
#include "RefereeTask.hpp"
#include "RosComm.hpp"
#include "RosTask.hpp"
#include "ShooterTask.hpp"
#include "VTMComm.hpp"
#include "gpio.h"
#include "main.h"
#include "task.h"

StackType_t uxBlinkTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t xBlinkTaskTCB;

void blink(void *pvPara)
{
    HAL_GPIO_WritePin(LED_ACT_GPIO_Port, LED_ACT_Pin, GPIO_PIN_RESET);
    while (true)
    {
        HAL_GPIO_TogglePin(LED_ACT_GPIO_Port, LED_ACT_Pin);
        vTaskDelay(500);
    }
}

/**
 * @brief Create user tasks
 */
void startUserTasks()
{
    Core::Drivers::CANManager::managers[0].init(&hfdcan1);
    Core::Drivers::CANManager::managers[1].init(&hfdcan2);
    Core::Drivers::CANManager::managers[2].init(&hfdcan3);

    Core::Communication::RefereeSystem::init();
    Core::Communication::RefereeSystem::initManager();
    Core::Drivers::VTMComm::init();
    Core::Drivers::DR16::init();
    Core::Drivers::IMU::init();
    Core::Drivers::IMU::setGyroOffset(-0.120f, -1.27f, 0.74f);

    Core::Drivers::Motors::DJIMotor::init();
    Core::Drivers::Motors::DMMotor::init();
    // Core::Drivers::Motors::LKMotor::init();
    Task::RefereeCommTask::init();
    Task::CommandTask::initCommandTask();
    Task::Shooter::init();
    Task::Gimbal::init();
    Task::RosTask::init();

    xTaskCreateStatic(blink, "blink", configMINIMAL_STACK_SIZE, NULL, 0, uxBlinkTaskStack, &xBlinkTaskTCB);
}

#pragma once
#include "RefereeSystemComm.hpp"
#include "RefereeSystemManager.hpp"
#include "RefereeSystemMessage.hpp"

namespace Task
{
namespace RefereeCommTask
{

struct RefereeData
{
    enum Color
    {
        RED,
        BLUE
    };

    Color robotColor;
    uint16_t robotID;
    uint8_t shootingFreq;
    uint16_t airSupportCoolingRemainingSeconds;
    uint16_t lastInitialSpeed;  // TODO: avg initial speed calculator
    uint16_t remainingAmmo;
    uint8_t radar_times;
};

const RefereeData &getRefereeData();

void init();

}  // namespace RefereeCommTask
}  // namespace Task
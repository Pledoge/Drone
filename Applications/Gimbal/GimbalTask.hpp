#include "GimbalController.hpp"

namespace Task
{
namespace Gimbal
{
void init();

Core::Drivers::Motors::J4310_2EC<AbstractFeedbackMotor::CommMode::OneToOne> &getPitchMotor();
const volatile Core::Control::Gimbal::EulerAngles &getCurrentGimbalEuler();
}  // namespace Gimbal
}  // namespace Task
#include "RosComm.hpp"
#include "main.h"

namespace Task
{
namespace RosTask
{
void init();
const volatile Core::Communication::RosComm::GimbalCommand &getRosGimbal();
}  // namespace RosTask
}  // namespace Task
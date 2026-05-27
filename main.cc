// #include "rosmodule.h"
#include "user_func.hpp"

int main(int argc, char **argv)
{
  // init ros
  // std::shared_ptr<RosStatusBridge> ros_bridge =
  //     std::make_shared<RosStatusBridge>();

  const char *kernel_config =
      argc > 1 ? argv[1] : BITBOT_DEFAULT_KERNEL_CONFIG;
  const char *controller_config =
      argc > 2 ? argv[2] : BITBOT_DEFAULT_CONTROLLER_CONFIG;

  MakeBitbotEverywhere everyone(kernel_config, controller_config);
  everyone.WillMake();
  everyone.BeMaking();
  everyone.HaveMade();
  return 0;
}

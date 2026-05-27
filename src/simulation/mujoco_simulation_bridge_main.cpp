#include "simulation/mujoco_simulation_bridge.h"

#include <array>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace
{
constexpr size_t kMotorCount = 24;

MujocoSimulationBridge *g_bridge = nullptr;

void SignalHandler(int signal)
{
  std::cout << "\n[MujocoSimulationBridge] received signal " << signal
            << ", stopping..." << std::endl;
  if (g_bridge)
  {
    g_bridge->Stop();
  }
}

const std::array<const char *, kMotorCount> kJointNames = {
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",

    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",

    "waist_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_elbow_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
    "head_joint",
};

// The values mirror config/efc.xml: motor index -> EfcJoint model_id.
const std::array<uint32_t, kMotorCount> kModelIds = {
    12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23,
    0,
    7, 8, 9, 10, 11,
    2, 3, 4, 5, 6,
    1,
};

std::map<std::string, size_t> CreateJointMapping()
{
  std::map<std::string, size_t> mapping;
  for (size_t i = 0; i < kJointNames.size(); ++i)
  {
    mapping[kJointNames[i]] = i;
  }
  return mapping;
}

std::map<size_t, std::string> CreateActuatorMapping()
{
  std::map<size_t, std::string> mapping;
  for (size_t i = 0; i < kJointNames.size(); ++i)
  {
    mapping[i] = kJointNames[i];
  }
  return mapping;
}

std::vector<uint32_t> CreateModelIdSequence()
{
  return {kModelIds.begin(), kModelIds.end()};
}
} // namespace

int main(int argc, char **argv)
{
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  std::cout << "========================================" << std::endl;
  std::cout << "MuJoCo simulation bridge for bitbot_efc v2" << std::endl;
  std::cout << "ROS subscribe : joint_states, imu_data" << std::endl;
  std::cout << "ROS publish   : actuators_cmds" << std::endl;
  std::cout << "DDS subscribe : RobotControlCommandTopic(domain 50)" << std::endl;
  std::cout << "DDS publish   : RobotDataTopic(domain 60)" << std::endl;
  std::cout << "========================================" << std::endl;

  MujocoSimulationBridge bridge;
  g_bridge = &bridge;
  bridge.Initialize(CreateJointMapping(), CreateActuatorMapping(),
                    CreateModelIdSequence());
  bridge.Run(argc, argv);
  g_bridge = nullptr;

  return 0;
}

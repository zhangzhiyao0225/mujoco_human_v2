#ifndef EFC_COMMON_HPP
#define EFC_COMMON_HPP

#include "device/efc_imu.h"
#include "device/efc_joint.h"
#include "kernel/efc_kernel.hpp"
#include "utils/efc_ankle.hpp"

namespace ovinf {

enum LeftRight {
  LEFT = 0,
  RIGHT = 1,
};

enum MotorIdx {
  LHipPitchMotor = 0,
  LHipRollMotor = 1,
  LHipYawMotor = 2,
  LKneePitchMotor = 3,
  LAnkleLongMotor = 4,
  LAnkleShortMotor = 5,

  RHipPitchMotor = 6,
  RHipRollMotor = 7,
  RHipYawMotor = 8,
  RKneePitchMotor = 9,
  RAnkleLongMotor = 10,
  RAnkleShortMotor = 11,

  WaistMotor = 12,

  LShoulderPitchMotor = 13,
  LShoulderRollMotor = 14,
  LShoulderYawMotor = 15,
  LElbowPitchMotor = 16,
  LElbowYawMotor = 17,

  RShoulderPitchMotor = 18,
  RShoulderRollMotor = 19,
  RShoulderYawMotor = 21,
  RElbowPitchMotor = 21,
  RElbowYawMotor = 22,

  HeadYawMotor = 23,
};

enum JointIdx {
  LHipPitchJoint = 0,
  LHipRollJoint = 1,
  LHipYawJoint = 2,
  LKneePitchJoint = 3,
  LAnklePitchJoint = 4,
  LAnkleRollJoint = 5,

  RHipPitchJoint = 6,
  RHipRollJoint = 7,
  RHipYawJoint = 8,
  RKneePitchJoint = 9,
  RAnklePitchJoint = 10,
  RAnkleRollJoint = 11,

  WaistJoint = 12,

  LShoulderPitchJoint = 13,
  LShoulderRollJoint = 14,
  LShoulderYawJoint = 15,
  LElbowPitchJoint = 16,
  LElbowYawJoint = 17,

  RShoulderPitchJoint = 18,
  RShoulderRollJoint = 19,
  RShoulderYawJoint = 21,
  RElbowPitchJoint = 21,
  RElbowYawJoint = 22,

  HeadYawJoint = 23,
};

}  // namespace ovinf

using KernelBus = bitbot::EfcBus;
using ImuDevice = bitbot::EfcImu;
using ImuPtr = ImuDevice*;
using MotorDevice = bitbot::EfcJoint;
using MotorPtr = MotorDevice*;
using AnkleT = ovinf::EfcAnkle<float>;
using AnklePtr = std::shared_ptr<AnkleT>;

struct UserData {};

using Kernel =
    bitbot::EfcKernel<UserData, "LAnklePitchPos", "LAnkleRollPos",
                      "RAnklePitchPos", "RAnkleRollPos", "LAnklePitchVel",
                      "LAnkleRollVel", "RAnklePitchVel", "RAnkleRollVel">;

#endif  // !EFC_COMMON_HPP

#include "device/efc_joint.h"

namespace bitbot
{

  EfcJoint::EfcJoint(const pugi::xml_node &device_node) : EfcDevice(device_node)
  {
    basic_type_ = (uint32_t)BasicDeviceType::MOTOR;
    type_ = (uint32_t)EfcDeviceType::EFC_JOINT;

    monitor_header_.headers = {
        "mode",
        "tar_q",
        "p_gain",
        "d_gain",
        "tar_tau",
        "act_q",
        "act_dq",
        "est_tau",
        "motor_t",
        "mos_t",
    };
    monitor_data_.resize(monitor_header_.headers.size());

    target_position_ = 0.0;

    ConfigParser::ParseAttribute2b(enable_, device_node.attribute("enable"));
    ConfigParser::ParseAttribute2ui(dds_motor_index_,
                                    device_node.attribute("dds_motor_index"));
    ConfigParser::ParseAttribute2ui(model_id_, device_node.attribute("model_id"));
    ConfigParser::ParseAttribute2d(ct_scale_, device_node.attribute("ct_scale"));
    ConfigParser::ParseAttribute2d(motor_dir_,
                                   device_node.attribute("motor_dir"));
    ConfigParser::ParseAttribute2d(pos_bias_, device_node.attribute("pos_bias"));
    ConfigParser::ParseAttribute2d(current_limit_,
                                   device_node.attribute("current_lim"));

    std::string mode_str;
    ConfigParser::ParseAttribute2s(mode_str, device_node.attribute("mode"));
    if (mode_str == "PFH")
    {
      joint_type_ = EfcJointType::PositionForceHybrid;
    }
    else
    {
      Logger().ConsoleLogger()->error(
          "Invalid joint mode: {}. Defaulting to NONE.", mode_str);
      joint_type_ = EfcJointType::NONE;
    }
  }

  EfcJoint::~EfcJoint() {}

  void EfcJoint::Input(const DdsInterface::Ptr dds_interface)
  {
    auto const &this_motor =
        dds_interface->GetRobotData()->motors()[dds_motor_index_];
    actual_position_ = motor_dir_ * this_motor.CurrentPosition() + pos_bias_;
    actual_velocity_ = this_motor.CurrentVelocity() * motor_dir_;
    actual_current_ = this_motor.CurrentCurrent() * motor_dir_;
    estimated_torque_ = actual_current_ * ct_scale_;
    motor_temp_ = this_motor.MotorTemperature();
    mos_temp_ = this_motor.DriverTemperature();
  }

  void EfcJoint::Output(const DdsInterface::Ptr dds_interface)
  {
    if (!enable_)
    {
      return;
    }

    auto &this_motor = dds_interface->GetCtrlCmd()->motors()[dds_motor_index_];
    this_motor.TargetPosition() = (target_position_ - pos_bias_) * motor_dir_;
    this_motor.Kp() = p_gain_;
    this_motor.Kd() = d_gain_;
    this_motor.TargetTorque() = target_torque_ * motor_dir_;

    // std::cout << "EfcJoint Output - Motor ID: " << this_motor.ModelId()
    //           << ", Target Pos: " << this_motor.TargetPosition()
    //           << ", Kp: " << this_motor.Kp() << ", Kd: " << this_motor.Kd()
    //           << ", Target Torque: " << this_motor.TargetTorque() << std::endl;
  }

  void EfcJoint::UpdateRuntimeData()
  {
    constexpr double rad2deg = 180.0 / M_PI;

    monitor_data_[0] = (int)joint_type_;
    monitor_data_[1] = rad2deg * target_position_;
    monitor_data_[2] = p_gain_;
    monitor_data_[3] = d_gain_;
    monitor_data_[4] = target_torque_;
    monitor_data_[5] = rad2deg * actual_position_;
    monitor_data_[6] = actual_velocity_;
    monitor_data_[7] = estimated_torque_;
    monitor_data_[8] = motor_temp_;
    monitor_data_[9] = mos_temp_;
  }

  void EfcJoint::UpdateModel(const DdsInterface::Ptr dds_interface)
  {
    for (size_t i = 0; i < dds_interface->GetRobotData()->motors().size(); ++i)
    {
      auto const &motor_data = dds_interface->GetRobotData()->motors()[i];
      if (motor_data.ModelId() == model_id_)
      {
        dds_motor_index_ = i;
        break;
      }
    }

    auto &this_motor = dds_interface->GetCtrlCmd()->motors()[dds_motor_index_];
    this_motor.enable() = enable_;
    this_motor.ModelId() = model_id_;
    this_motor.CurrentLimit() = current_limit_;

    this_motor.mode() = 0;
    // Action:
    // 100 - IDLE
    // 101 - RESET
    // 102 - MOVE
    this_motor.action() = 100;

    // Init control target
    this_motor.TargetPosition() = 0.0;
    this_motor.Kp() = 0.0;
    this_motor.Kd() = 0.0;
    this_motor.TargetTorque() = 0.0;
    this_motor.TargetVelocity() = 0.0;
    this_motor.TargetCurrent() = 0.0;
  }

} // namespace bitbot

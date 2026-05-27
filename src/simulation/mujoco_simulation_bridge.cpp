#include "simulation/mujoco_simulation_bridge.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
uint64_t NowMicros()
{
  const auto now = std::chrono::system_clock::now();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          now.time_since_epoch())
          .count());
}
} // namespace

MujocoSimulationBridge::~MujocoSimulationBridge()
{
  Stop();
}

void MujocoSimulationBridge::Initialize(
    const std::map<std::string, size_t> &joint_name_mapping,
    const std::map<size_t, std::string> &actuator_name_mapping,
    const std::vector<uint32_t> &model_id_sequence)
{
  joint_name_to_motor_idx_ = joint_name_mapping;
  motor_idx_to_actuator_name_ = actuator_name_mapping;
  motor_model_ids_ = model_id_sequence;

  if (motor_model_ids_.size() < kMotorCount)
  {
    std::cout << "[MujocoSimulationBridge] ModelId count "
              << motor_model_ids_.size() << " < " << kMotorCount
              << ", padding with index values" << std::endl;
    for (size_t i = motor_model_ids_.size(); i < kMotorCount; ++i)
    {
      motor_model_ids_.push_back(static_cast<uint32_t>(i));
    }
  }
  else if (motor_model_ids_.size() > kMotorCount)
  {
    motor_model_ids_.resize(kMotorCount);
  }

  model_id_to_motor_idx_.clear();
  for (size_t motor_idx = 0; motor_idx < motor_model_ids_.size(); ++motor_idx)
  {
    model_id_to_motor_idx_[motor_model_ids_[motor_idx]] = motor_idx;
  }

  initialized_ = true;
  std::cout << "[MujocoSimulationBridge] initialized with "
            << joint_name_to_motor_idx_.size() << " joints" << std::endl;
}

void MujocoSimulationBridge::Run()
{
  Start();

  constexpr auto publish_period = std::chrono::microseconds(2000);
  while (running_.load() && rclcpp::ok())
  {
    PublishRobotData();
    std::this_thread::sleep_for(publish_period);
  }

  Stop();
}

void MujocoSimulationBridge::Run(int argc, char **argv)
{
  Start(argc, argv);

  constexpr auto publish_period = std::chrono::microseconds(2000);
  while (running_.load() && rclcpp::ok())
  {
    PublishRobotData();
    std::this_thread::sleep_for(publish_period);
  }

  Stop();
}

void MujocoSimulationBridge::Start(int argc, char **argv)
{
  if (!initialized_)
  {
    std::cerr << "[MujocoSimulationBridge] initialize before Start/Run"
              << std::endl;
    return;
  }
  if (running_.exchange(true))
  {
    return;
  }

  if (!rclcpp::ok())
  {
    rclcpp::init(argc, argv);
  }

  ros_node_ = std::make_shared<rclcpp::Node>("mujoco_simulation_bridge");
  joint_state_topic_ =
      ros_node_->declare_parameter<std::string>("joint_state_topic",
                                                joint_state_topic_);
  imu_topic_ = ros_node_->declare_parameter<std::string>("imu_topic",
                                                         imu_topic_);
  actuator_cmd_topic_ =
      ros_node_->declare_parameter<std::string>("actuator_cmd_topic",
                                                actuator_cmd_topic_);
  robot_data_domain_id_ =
      ros_node_->declare_parameter<int>("robot_data_domain_id",
                                        robot_data_domain_id_);
  robot_command_domain_id_ =
      ros_node_->declare_parameter<int>("robot_command_domain_id",
                                        robot_command_domain_id_);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  joint_state_sub_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, qos,
      std::bind(&MujocoSimulationBridge::JointStateCallback, this,
                std::placeholders::_1));
  imu_sub_ = ros_node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      std::bind(&MujocoSimulationBridge::ImuCallback, this,
                std::placeholders::_1));
  actuator_cmd_pub_ =
      ros_node_->create_publisher<custom_msgs::msg::ActuatorCmds>(
          actuator_cmd_topic_, qos);

  robot_data_publisher_ = std::make_shared<RobotDataPublisher>(
      robot_data_domain_id_, "Participant_MujocoBridge_DataPub",
      "RobotDataTopic");
  robot_command_subscriber_ = std::make_shared<RobotCommandSubscriber>(
      robot_command_domain_id_, "Participant_MujocoBridge_CommandSub",
      "RobotControlCommandTopic");

  ros_spin_thread_ = std::thread(&MujocoSimulationBridge::RosSpinThread, this);
  dds_command_thread_ =
      std::thread(&MujocoSimulationBridge::DdsCommandThread, this);

  std::cout << "[MujocoSimulationBridge] running" << std::endl;
  std::cout << "  ROS subscribe : " << joint_state_topic_ << ", "
            << imu_topic_ << std::endl;
  std::cout << "  ROS publish   : " << actuator_cmd_topic_ << std::endl;
  std::cout << "  DDS publish   : RobotDataTopic(domain "
            << robot_data_domain_id_ << ")" << std::endl;
  std::cout << "  DDS subscribe : RobotControlCommandTopic(domain "
            << robot_command_domain_id_ << ")" << std::endl;
}

void MujocoSimulationBridge::Stop()
{
  if (!running_.exchange(false))
  {
    return;
  }

  if (ros_node_ && rclcpp::ok())
  {
    rclcpp::shutdown();
  }
  if (ros_spin_thread_.joinable())
  {
    ros_spin_thread_.join();
  }
  if (dds_command_thread_.joinable())
  {
    dds_command_thread_.join();
  }

  robot_command_subscriber_.reset();
  robot_data_publisher_.reset();
  actuator_cmd_pub_.reset();
  imu_sub_.reset();
  joint_state_sub_.reset();
  ros_node_.reset();

  std::cout << "[MujocoSimulationBridge] stopped" << std::endl;
}

void MujocoSimulationBridge::RosSpinThread()
{
  rclcpp::spin(ros_node_);
}

void MujocoSimulationBridge::DdsCommandThread()
{
  using namespace std::chrono_literals;

  while (running_.load())
  {
    if (robot_command_subscriber_ &&
        robot_command_subscriber_->MessageAvailable())
    {
      auto msg = robot_command_subscriber_->GetMessage();
      if (msg)
      {
        if (!printed_dds_command_)
        {
          printed_dds_command_ = true;
          std::cout << "[MujocoSimulationBridge] first DDS RobotControlCommand received"
                    << std::endl;
        }
        PublishActuatorCommands(*msg);
      }
    }
    std::this_thread::sleep_for(1ms);
  }
}

void MujocoSimulationBridge::JointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_joint_state_ = *msg;
  has_joint_state_ = true;
  if (!printed_joint_state_)
  {
    printed_joint_state_ = true;
    std::cout << "[MujocoSimulationBridge] first ROS JointState received: "
              << msg->name.size() << " joints from " << joint_state_topic_
              << std::endl;
  }
}

void MujocoSimulationBridge::ImuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_imu_ = *msg;
  has_imu_ = true;
  if (!printed_imu_)
  {
    printed_imu_ = true;
    std::cout << "[MujocoSimulationBridge] first ROS Imu received from "
              << imu_topic_ << std::endl;
  }
}

void MujocoSimulationBridge::PublishRobotData()
{
  if (!robot_data_publisher_)
  {
    return;
  }

  sensor_msgs::msg::JointState joint_state;
  sensor_msgs::msg::Imu imu;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_joint_state_ || !has_imu_)
    {
      return;
    }
    joint_state = latest_joint_state_;
    imu = latest_imu_;
  }

  auto robot_data = std::make_shared<RobotData>();
  robot_data->timestamp(NowMicros());

  std::array<MotorState, kMotorCount> motor_states{};
  for (auto &state : motor_states)
  {
    state.MotorTemperature(25.0f);
    state.DriverTemperature(25.0f);
  }

  for (size_t msg_idx = 0; msg_idx < joint_state.name.size(); ++msg_idx)
  {
    const auto mapping_it = joint_name_to_motor_idx_.find(joint_state.name[msg_idx]);
    if (mapping_it == joint_name_to_motor_idx_.end())
    {
      continue;
    }

    const size_t motor_idx = mapping_it->second;
    if (motor_idx >= kMotorCount)
    {
      continue;
    }

    const float position =
        msg_idx < joint_state.position.size()
            ? static_cast<float>(joint_state.position[msg_idx])
            : 0.0f;
    const float velocity =
        msg_idx < joint_state.velocity.size()
            ? static_cast<float>(joint_state.velocity[msg_idx])
            : 0.0f;
    const float effort =
        msg_idx < joint_state.effort.size()
            ? static_cast<float>(joint_state.effort[msg_idx])
            : 0.0f;

    auto &state = motor_states[motor_idx];
    state.CurrentPosition(position);
    state.CurrentVelocity(velocity);
    state.CurrentCurrent(effort);
    state.TargetPosition(position);
    state.TargetVelocity(0.0f);
    state.TargetCurrent(0.0f);
    state.JointMode(0);
    state.CanId(static_cast<uint32_t>(motor_idx));
    state.EtherCatId(static_cast<uint32_t>(motor_idx));
    state.ModelId(GetModelIdForMotor(motor_idx));
  }

  robot_data->motors(std::move(motor_states));

  tf2::Quaternion quat(imu.orientation.x, imu.orientation.y, imu.orientation.z,
                       imu.orientation.w);
  tf2::Matrix3x3 rot(quat);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  rot.getRPY(roll, pitch, yaw);

  IMUData imu_data;
  imu_data.ImuId(0);
  imu_data.Roll(static_cast<float>(roll));
  imu_data.Pitch(static_cast<float>(pitch));
  imu_data.Yaw(static_cast<float>(yaw));
  imu_data.A_x(static_cast<float>(imu.linear_acceleration.x));
  imu_data.A_y(static_cast<float>(imu.linear_acceleration.y));
  imu_data.A_z(static_cast<float>(imu.linear_acceleration.z));
  imu_data.W_x(static_cast<float>(imu.angular_velocity.x));
  imu_data.W_y(static_cast<float>(imu.angular_velocity.y));
  imu_data.W_z(static_cast<float>(imu.angular_velocity.z));
  imu_data.ImuTemp(25.0f);

  std::array<IMUData, 2> imu_datas{imu_data, imu_data};
  imu_datas[1].ImuId(1);
  robot_data->imu(std::move(imu_datas));
  robot_data->battery_voltage(48.0f);
  robot_data->emergency_stop(false);

  robot_data_publisher_->Publish(robot_data);
  if (!printed_robot_data_)
  {
    printed_robot_data_ = true;
    std::cout << "[MujocoSimulationBridge] first DDS RobotData published"
              << std::endl;
  }
}

void MujocoSimulationBridge::PublishActuatorCommands(
    const RobotControlCommand &cmd)
{
  if (!actuator_cmd_pub_)
  {
    return;
  }

  custom_msgs::msg::ActuatorCmds actuator_cmds;
  for (const auto &motor_cmd : cmd.motors())
  {
    if (!motor_cmd.enable())
    {
      continue;
    }

    const auto model_it = model_id_to_motor_idx_.find(motor_cmd.ModelId());
    if (model_it == model_id_to_motor_idx_.end())
    {
      continue;
    }

    const auto actuator_it = motor_idx_to_actuator_name_.find(model_it->second);
    if (actuator_it == motor_idx_to_actuator_name_.end())
    {
      continue;
    }

    actuator_cmds.actuators_name.push_back(actuator_it->second);
    actuator_cmds.pos.push_back(motor_cmd.TargetPosition());
    actuator_cmds.vel.push_back(motor_cmd.TargetVelocity());
    actuator_cmds.kp.push_back(motor_cmd.Kp());
    actuator_cmds.kd.push_back(motor_cmd.Kd());
    actuator_cmds.torque.push_back(motor_cmd.TargetTorque());
    actuator_cmds.torque_limit.push_back(
        motor_cmd.CurrentLimit() > 0.0f ? motor_cmd.CurrentLimit() : 200.0f);
  }

  if (!actuator_cmds.actuators_name.empty())
  {
    actuator_cmd_pub_->publish(actuator_cmds);
  }
}

uint32_t MujocoSimulationBridge::GetModelIdForMotor(size_t motor_idx) const
{
  if (motor_idx < motor_model_ids_.size())
  {
    return motor_model_ids_[motor_idx];
  }
  return static_cast<uint32_t>(motor_idx);
}

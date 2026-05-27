#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "RobotData/RobotDataPubSubTypes.h"
#include "RobotRecvData/RobotRecvDataPubSubTypes.h"
#include "custom_msgs/msg/actuator_cmds.hpp"
#include "dds/dds_pubsub.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MujocoSimulationBridge
{
public:
  MujocoSimulationBridge() = default;
  ~MujocoSimulationBridge();

  void Initialize(const std::map<std::string, size_t> &joint_name_mapping,
                  const std::map<size_t, std::string> &actuator_name_mapping,
                  const std::vector<uint32_t> &model_id_sequence);

  void Run();
  void Run(int argc, char **argv);
  void Stop();

private:
  using RobotDataPublisher = bitbot::DdsPublisher<RobotDataPubSubType>;
  using RobotCommandSubscriber = bitbot::DdsSubscriber<RobotControlCommandPubSubType>;

  void Start(int argc = 0, char **argv = nullptr);
  void RosSpinThread();
  void DdsCommandThread();
  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void PublishRobotData();
  void PublishActuatorCommands(const RobotControlCommand &cmd);
  uint32_t GetModelIdForMotor(size_t motor_idx) const;

private:
  static constexpr size_t kMotorCount = 24;

  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr actuator_cmd_pub_;

  std::shared_ptr<RobotDataPublisher> robot_data_publisher_;
  std::shared_ptr<RobotCommandSubscriber> robot_command_subscriber_;

  std::map<std::string, size_t> joint_name_to_motor_idx_;
  std::map<size_t, std::string> motor_idx_to_actuator_name_;
  std::vector<uint32_t> motor_model_ids_;
  std::map<uint32_t, size_t> model_id_to_motor_idx_;

  sensor_msgs::msg::JointState latest_joint_state_;
  sensor_msgs::msg::Imu latest_imu_;
  std::mutex data_mutex_;
  bool has_joint_state_{false};
  bool has_imu_{false};
  bool printed_joint_state_{false};
  bool printed_imu_{false};
  bool printed_dds_command_{false};
  bool printed_robot_data_{false};

  std::string joint_state_topic_{"joint_states"};
  std::string imu_topic_{"imu_data"};
  std::string actuator_cmd_topic_{"actuators_cmds"};
  int robot_data_domain_id_{60};
  int robot_command_domain_id_{50};

  std::thread ros_spin_thread_;
  std::thread dds_command_thread_;
  std::atomic_bool running_{false};
  bool initialized_{false};
};

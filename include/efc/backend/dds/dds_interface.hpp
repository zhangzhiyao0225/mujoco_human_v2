#ifndef DDS_INTERFACE_HPP
#define DDS_INTERFACE_HPP

#include <map>
#include <pugixml.hpp>
#include <thread>

#include "RobotData/RobotDataPubSubTypes.h"
#include "RobotRecvData/RobotRecvDataPubSubTypes.h"
#include "dds/dds_pubsub.hpp"

namespace bitbot
{

  using namespace std::chrono_literals;

  class DdsInterface
  {
    using SubscriberType = DdsSubscriber<RobotDataPubSubType>;
    using PublisherType = DdsPublisher<RobotControlCommandPubSubType>;

  public:
    using Ptr = std::shared_ptr<DdsInterface>;
    DdsInterface(const pugi::xml_node &config_node)
    {
      pugi::xml_node subscriber_node = config_node.child("subscriber");
      pugi::xml_node publisher_node = config_node.child("publisher");

      robot_data_subscriber_ = std::make_shared<SubscriberType>(
          subscriber_node.attribute("domain_id").as_int(),
          subscriber_node.attribute("participant_name").as_string(),
          subscriber_node.attribute("topic_name").as_string());
      ctrl_cmd_publisher_ = std::make_shared<PublisherType>(
          publisher_node.attribute("domain_id").as_int(),
          publisher_node.attribute("participant_name").as_string(),
          publisher_node.attribute("topic_name").as_string());
      ctrl_cmd_msg_ = std::make_shared<RobotControlCommand>();
    }

    ~DdsInterface() {}

    void PublishJointCommand()
    {

    // static int i = 0;
    // if (++i == 100)
    // {
	  //   i = 0;
	  //   std::cout << "publish target pos into motor " << publishing_enabled_.load() << " @ " << motor_reset_done_ << std::endl;
    // }

      if (publishing_enabled_.load())
      {
        ctrl_cmd_publisher_->Publish(ctrl_cmd_msg_);
      }

    }

    SubscriberType::MsgPtr GetRobotData() { return robot_data_msg_; }

    void UpdateRobotData()
    {
      robot_data_msg_ = robot_data_subscriber_->GetMessage();
    }

    PublisherType::MsgPtr GetCtrlCmd() { return ctrl_cmd_msg_; }

    void WaitingForSystemReady()
    {
      while (!robot_data_subscriber_->MessageAvailable())
      {
        std::this_thread::sleep_for(100ms);
      }

      system_ready_.store(true);
    }

    bool EnterDampingMode()
    {
      for (auto &motor_cmd : ctrl_cmd_msg_->motors())
      {
        motor_cmd.TargetPosition() = 0.0;
        motor_cmd.TargetVelocity() = 0.0;
        motor_cmd.TargetTorque() = 0.0;
        motor_cmd.TargetCurrent() = 0.0;
        motor_cmd.Kp() = 0.0;
        motor_cmd.Kd() = 1.0;
      }
      ctrl_cmd_publisher_->Publish(ctrl_cmd_msg_);
      return true;
    }

    bool ShutDown()
    {
      if (motor_reset_done_)
      {
        for (auto &motor_cmd : ctrl_cmd_msg_->motors())
        {
          motor_cmd.TargetPosition() = 0.0;
          motor_cmd.TargetVelocity() = 0.0;
          motor_cmd.TargetTorque() = 0.0;
          motor_cmd.TargetCurrent() = 0.0;
          motor_cmd.Kp() = 0.0;
          motor_cmd.Kd() = 0.0;
        }
        ctrl_cmd_publisher_->Publish(ctrl_cmd_msg_);
        std::this_thread::sleep_for(100ms);
        ctrl_cmd_publisher_->Publish(ctrl_cmd_msg_);
        std::this_thread::sleep_for(100ms);
        ctrl_cmd_publisher_->Publish(ctrl_cmd_msg_);
      }
      return true;
    }

    bool MotorReset()
    {
      for (auto &motor_cmd : ctrl_cmd_msg_->motors())
      {
        motor_cmd.action() = 101;
      }
      ctrl_cmd_publisher_->Publish(ctrl_cmd_msg_);
      motor_reset_done_ = true;
      return true;
    }

    bool MotorCostomMode()
    {
      if (motor_reset_done_)
      {
        for (auto &motor_cmd : ctrl_cmd_msg_->motors())
        {
          motor_cmd.action() = 102;
        }
        ctrl_cmd_publisher_->Publish(ctrl_cmd_msg_);
        publishing_enabled_.store(true);
        return true;
      }
      else
      {
        return false;
      }
    }

  private:
    // DDS members
    SubscriberType::SubscriberPtr robot_data_subscriber_;
    PublisherType::PublisherPtr ctrl_cmd_publisher_;

    SubscriberType::MsgPtr robot_data_msg_{nullptr};
    PublisherType::MsgPtr ctrl_cmd_msg_{nullptr};

    std::atomic_bool system_ready_{false};
    std::atomic_bool publishing_enabled_{false};

    bool motor_reset_done_{false};
  };

} // namespace bitbot

#endif // !EFC_NODE_HPP

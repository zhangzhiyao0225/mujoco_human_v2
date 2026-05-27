#include "status_publisher.hpp"
#include "data_center/DataCenter.h"
#include "LogDefine.h"
StatusPublisher::StatusPublisher(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    robot_pub_ = node_->create_publisher<std_msgs::msg::String>("/RobotHuman_Info", 10);

    alarm_pub_ = node_->create_publisher<std_msgs::msg::String>("/RobotHuman_Alarm", 10);

    heart_pub_ = node_->create_publisher<std_msgs::msg::String>("/RobotHuman_Heart", 10);

    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100), // 10Hz
        std::bind(&StatusPublisher::publish_robot_state, this));
}

void StatusPublisher::publish_robot_state()
{
    //状态信息
    std::string data_ = DataCenterInstance->Serialize();
    MechineLOG_INFO(data_);
    std_msgs::msg::String msg;
    msg.data = data_;
    robot_pub_->publish(msg);

    //报警信息
    std::string alarm_ = DataCenterInstance->GetRobot_Alarm();
    std_msgs::msg::String msg_alarm;
    msg_alarm.data = alarm_;
    RosLOG_ERROR(alarm_);
    alarm_pub_->publish(msg_alarm);
    DataCenterInstance->SetRobot_Alarm("");

    //心跳信息
    std::string heart_ = DataCenterInstance->GetRobot_Heart();
    std_msgs::msg::String msg_heart;
    msg_heart.data = heart_;
    heart_pub_->publish(msg_heart);
}

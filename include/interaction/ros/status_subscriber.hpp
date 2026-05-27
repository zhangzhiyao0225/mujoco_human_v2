#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class StatusSubscriber
{
public:
    StatusSubscriber(rclcpp::Node::SharedPtr node);
    ~StatusSubscriber() = default;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_action_;

    // 回调函数
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void robotActionCallback(const std_msgs::msg::Int32::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr imu_timer_;
};

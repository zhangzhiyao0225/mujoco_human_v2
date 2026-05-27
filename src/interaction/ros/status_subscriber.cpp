#include "status_subscriber.hpp"
#include "data_center/DataCenter.h"

StatusSubscriber::StatusSubscriber(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/robot_cmd_vel", 10, std::bind(&StatusSubscriber::cmdVelCallback, this, std::placeholders::_1));

    robot_action_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot_action", 10, std::bind(&StatusSubscriber::robotActionCallback, this, std::placeholders::_1));

}

void StatusSubscriber::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    DataCenterInstance->UpdateCmdVel(*msg);
}


void StatusSubscriber::robotActionCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int action = msg->data;
    
    DataCenterInstance->UpdateAction(action);
}

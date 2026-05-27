#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/imu.hpp"                 
#include "sensor_msgs/msg/magnetic_field.hpp"      
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

class StatusPublisher
{
public:
    StatusPublisher(rclcpp::Node::SharedPtr node);
    ~StatusPublisher() = default;

    void publish_robot_state();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alarm_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heart_pub_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr timer_;
};

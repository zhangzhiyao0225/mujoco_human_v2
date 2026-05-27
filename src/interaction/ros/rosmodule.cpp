#include "rosmodule.h"
#include "LogDefine.h"

RosStatusBridge::RosStatusBridge()
{
    int argc = 0;
    char **argv= nullptr;
    // 初始化ROS2
    if (!rclcpp::ok())
        rclcpp::init(argc, argv);

    // 创建节点
    node_ = std::make_shared<rclcpp::Node>("number_twin_bridge");

    // 初始化你的订阅器
    subscriber_ = std::make_shared<StatusSubscriber>(node_);
    publisher_ = std::make_shared<StatusPublisher>(node_);

    // 独立线程spin
    spin_thread_ = std::thread([this]() {
        rclcpp::spin(node_);
    });

}

RosStatusBridge::~RosStatusBridge()
{
}

void RosStatusBridge::shutdown()
{
    if (spin_thread_.joinable())
    {
        node_->get_node_base_interface()->get_context()->shutdown("RosStatusBridge");
        spin_thread_.join();
    }
}

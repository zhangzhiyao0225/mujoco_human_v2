#ifndef ROSMODULE_H
#define ROSMODULE_H

// RosStatusBridge.hpp

#pragma once

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "status_publisher.hpp"
#include "status_subscriber.hpp"

class RosStatusBridge
{
public:
    RosStatusBridge();
    ~RosStatusBridge();

    void shutdown();

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<StatusSubscriber> subscriber_;
    std::shared_ptr<StatusPublisher> publisher_;
    std::thread spin_thread_;
};

#endif // ROSMODULE_H

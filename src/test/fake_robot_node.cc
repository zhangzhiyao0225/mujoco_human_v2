#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/srv/rpc_service.hpp"
#include "rclcpp/rclcpp.hpp"

class FakeBoosterNode : public rclcpp::Node {
 public:
  FakeBoosterNode() : rclcpp::Node("fake_booster_node") {
    low_state_msg_.motor_state_parallel.resize(23);
    low_state_msg_.motor_state_serial.resize(23);

    low_state_pub_ = this->create_publisher<booster_interface::msg::LowState>(
        "/low_state", 10);
    low_cmd_sub_ = this->create_subscription<booster_interface::msg::LowCmd>(
        "/joint_ctrl", 10,
        std::bind(&FakeBoosterNode::LowCmdCallback, this,
                  std::placeholders::_1));
    rpc_service_ = this->create_service<booster_interface::srv::RpcService>(
        "/booster_rpc_service",
        std::bind(&FakeBoosterNode::RpcServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&FakeBoosterNode::PublishLowState, this));
  }

 private:
  void LowCmdCallback(const booster_interface::msg::LowCmd::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received LowCmd with %zu motor
    // commands", msg->motor_cmd.size());
    for (size_t i = 0; i < 23; ++i) {
      if (robot_mode_ == 3) {
        if (msg->cmd_type ==
            booster_interface::msg::LowCmd::CMD_TYPE_PARALLEL) {
          double kp = msg->motor_cmd[i].kp;
          double kd = msg->motor_cmd[i].kd;
          low_state_msg_.motor_state_parallel[i].tau_est =
              kp * (msg->motor_cmd[i].q -
                    low_state_msg_.motor_state_parallel[i].q) -
              kd * low_state_msg_.motor_state_parallel[i].dq;
          low_state_msg_.motor_state_parallel[i].q = msg->motor_cmd[i].q;
        } else if (msg->cmd_type ==
                   booster_interface::msg::LowCmd::CMD_TYPE_SERIAL) {
          double kp = msg->motor_cmd[i].kp;
          double kd = msg->motor_cmd[i].kd;
          low_state_msg_.motor_state_serial[i].tau_est =
              kp * (msg->motor_cmd[i].q -
                    low_state_msg_.motor_state_serial[i].q) -
              kd * low_state_msg_.motor_state_serial[i].dq;
          low_state_msg_.motor_state_serial[i].q = msg->motor_cmd[i].q;
        }
      } else {
        low_state_msg_.motor_state_parallel[i].tau_est = 0.0;
        low_state_msg_.motor_state_serial[i].tau_est = 0.0;
      }
    }
  }

  void PublishLowState() { low_state_pub_->publish(low_state_msg_); }

  void RpcServiceCallback(
      const std::shared_ptr<booster_interface::srv::RpcService::Request>
          request,
      std::shared_ptr<booster_interface::srv::RpcService::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received RPC request: %s",
                request->msg.body.c_str());
    if (request->msg.body == "{\"mode\":0}") {
      robot_mode_ = 0;
    } else if (request->msg.body == "{\"mode\":1}") {
      robot_mode_ = 1;
    } else if (request->msg.body == "{\"mode\":2}") {
      robot_mode_ = 2;
    } else if (request->msg.body == "{\"mode\":3}") {
      robot_mode_ = 3;
    }

    // Respond with a simple acknowledgment
    response->msg.body = "RPC request processed successfully.";
  }

 private:
  rclcpp::Publisher<booster_interface::msg::LowState>::SharedPtr low_state_pub_;
  rclcpp::Subscription<booster_interface::msg::LowCmd>::SharedPtr low_cmd_sub_;
  rclcpp::Service<booster_interface::srv::RpcService>::SharedPtr rpc_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  booster_interface::msg::LowState low_state_msg_;

  unsigned int robot_mode_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeBoosterNode>());

  rclcpp::shutdown();
  return 0;
}

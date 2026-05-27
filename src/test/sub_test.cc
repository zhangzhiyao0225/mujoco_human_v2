#include "RobotData/RobotDataPubSubTypes.h"
#include "RobotRecvData/RobotRecvDataPubSubTypes.h"
#include "dds/dds_pubsub.hpp"

int main() {
  constexpr int pub_domain_id{21};
  constexpr std::string_view pub_participant_name{"TestPublisher"};
  constexpr std::string_view pub_topic_name{"RobotControlCommandTopic"};
  auto publisher =
      std::make_shared<bitbot::DdsPublisher<RobotControlCommandPubSubType>>(
          pub_domain_id, pub_participant_name.data(), pub_topic_name.data());

  constexpr int sub_domain_id{20};
  constexpr std::string_view sub_participant_name{"TestSubscriber"};
  constexpr std::string_view sub_topic_name{"RobotDataTopic"};
  auto subscriber =
      std::make_shared<bitbot::DdsSubscriber<RobotDataPubSubType>>(
          sub_domain_id, sub_participant_name.data(), sub_topic_name.data());

  int counter = 0;
  while (true) {
    if (subscriber->MessageAvailable()) {
      auto msg = subscriber->GetMessage();
      // Process the received MotorCommand message
      // For example, print out the ModelId of the first motor command
      if (!msg->motors().empty()) {
        // std::cout << "Received RobotData." << std::endl;
      }
    }

    if (counter++ % 100 == 0) {
      auto cmd_msg = std::make_shared<RobotControlCommand>();
      // Populate cmd_msg with data
      for (size_t i = 0; i < cmd_msg->motors().size(); ++i) {
        auto& motor_cmd = cmd_msg->motors()[i];
        motor_cmd.enable() = true;
        motor_cmd.ModelId() = static_cast<uint32_t>(i);
        motor_cmd.TargetPosition() = 1.0f * i;
        motor_cmd.TargetVelocity() = 0.5f * i;
      }
      cmd_msg->timestamp() = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count());
      publisher->Publish(cmd_msg);
      std::cout << "Published RobotControlCommand." << std::endl;
    }
  }
  return 0;
}

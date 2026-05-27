#include "RobotData/RobotDataPubSubTypes.h"
#include "RobotRecvData/RobotRecvDataPubSubTypes.h"
#include "dds/dds_pubsub.hpp"

constexpr int pub_domain_id{60};
constexpr std::string_view pub_participant_name{"FakeRobotPublisher"};
constexpr std::string_view pub_topic_name{"RobotDataTopic"};
constexpr int sub_domain_id{50};
constexpr std::string_view sub_participant_name{"FakeRobotSubscriber"};
constexpr std::string_view sub_topic_name{"RobotControlCommandTopic"};
constexpr size_t motor_count{24};

std::map<int, int> model_id_map = {
    {0, 2},   {1, 3},   {2, 4},   {3, 5},   {4, 6},   {5, 7},
    {6, 8},   {7, 9},   {8, 10},  {9, 11},  {10, 1},  {11, 0},
    {12, 18}, {13, 19}, {14, 20}, {15, 21}, {16, 22}, {17, 23},
    {18, 12}, {19, 13}, {20, 14}, {21, 15}, {22, 16}, {23, 17},
};

int main() {
  auto publisher = std::make_shared<bitbot::DdsPublisher<RobotDataPubSubType>>(
      pub_domain_id, pub_participant_name.data(), pub_topic_name.data());

  auto subscriber =
      std::make_shared<bitbot::DdsSubscriber<RobotControlCommandPubSubType>>(
          sub_domain_id, sub_participant_name.data(), sub_topic_name.data());

  auto robot_data_msg = std::make_shared<RobotData>();
  size_t counter = 0;

  for (size_t i = 0; i < motor_count; ++i) {
    auto& data_motor = robot_data_msg->motors()[i];
    data_motor.ModelId() = model_id_map[i];
  }

  bool reset_flag = false;
  bool custom_mode_flag = false;

  while (true) {
    if (subscriber->MessageAvailable()) {
      auto cmd_msg = subscriber->GetMessage();
      for (size_t i = 0; i < motor_count; ++i) {
        auto& data_motor = robot_data_msg->motors()[i];
        auto& cmd_motor = cmd_msg->motors()[i];

        if (data_motor.ModelId() != cmd_motor.ModelId()) {
          std::cout << "Model ID mismatch! correct: "
                    << data_motor.ModelId(

                           )
                    << std::endl;
        }

        if (i == 0 && !reset_flag) {
          if (cmd_motor.action() == 101) {
            std::cout << "Received motor reset command." << std::endl;
            reset_flag = true;
          }
        }

        if (i == 0 && reset_flag && !custom_mode_flag) {
          if (cmd_motor.action() == 102) {
            std::cout << "Received motor custom mode command." << std::endl;
            custom_mode_flag = true;
          }
        }

        float last_position = data_motor.CurrentPosition();

        float total_torque = cmd_motor.TargetTorque() +
                             cmd_motor.Kp() * (cmd_motor.TargetPosition() -
                                               data_motor.CurrentPosition()) -
                             cmd_motor.Kd() * data_motor.CurrentVelocity();

        if (cmd_motor.ModelId() == 16 || cmd_motor.ModelId() == 17 ||
            cmd_motor.ModelId() == 22 || cmd_motor.ModelId() == 23) {
          std::cout << "Motor ID: " << cmd_motor.ModelId() << std::endl;
          std::cout << "Target pos: " << cmd_motor.TargetPosition()
                    << ", Current pos: " << data_motor.CurrentPosition()
                    << std::endl;
          std::cout << "Target vel: " << cmd_motor.TargetVelocity()
                    << ", Current vel: " << data_motor.CurrentVelocity()
                    << std::endl;
          std::cout << "Target torque: " << cmd_motor.TargetTorque()
                    << std::endl;

          std::cout << "Total torque: " << total_torque << std::endl;
        }

        // Fake dynamics
        data_motor.CurrentPosition() = total_torque * 0.005 + last_position;
        data_motor.CurrentVelocity() = std::max<float>(
            std::min<float>(
                (data_motor.CurrentPosition() - last_position) * 100.0f, 3.0),
            -3.0);

        // data_motor.CurrentVelocity() = 0.0;
      }

      if (counter++ % 500 == 0) {
        // 1Hz printout
        std::cout << "Received RobotControlCommand. " << counter << std::endl;
        std::cout << robot_data_msg->motors()[0].CurrentPosition() << ", "
                  << robot_data_msg->motors()[0].CurrentVelocity() << std::endl;
      }
    }
    robot_data_msg->timestamp() = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    publisher->Publish(robot_data_msg);

    // 500 Hz
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  return 0;
}

#ifndef POLICY_CTR_BEYOND_MIMIC_HPP
#define POLICY_CTR_BEYOND_MIMIC_HPP

#include <iostream>
#include <map>
#include <string>

#include "policy_controller_base.hpp"

namespace ovinf {

class PolicyCtrBeyondMimic : public PolicyControllerBase {
 public:
  using Ptr = std::shared_ptr<PolicyCtrBeyondMimic>;

  PolicyCtrBeyondMimic() = delete;
  ~PolicyCtrBeyondMimic() = default;

  PolicyCtrBeyondMimic(RobotBase<float>::RobotPtr robot,
                       YAML::Node const& config)
      : PolicyControllerBase(robot, config) {
    action_size_ = config["inference"]["action_size"].as<size_t>();
    auto_start_ = config["inference"]["auto_start"].as<bool>(false);

    for (size_t i = 0; i < action_size_; ++i) {
      const std::string joint_name =
          config["inference"]["policy_joint_names"][i].as<std::string>();
      policy_joint_idx_map_[i] = robot_->joint_names_.at(joint_name);
    }
  }

  void WarmUp() final {
    if (counter_++ % decimation_ == 0) {
      inference_net_->WarmUp(MakeObservation());
    }

    policy_target_position_ = robot_->Executor()->JointTargetPosition();
    if (target_pos_filter_) {
      target_pos_filter_->Filter(policy_target_position_);
    }
  }

  void Init() final {
    counter_ = 0;
    command_ = VectorT::Zero(3);
    ready_ = true;
    policy_started_ = auto_start_;
    hold_position_ = robot_->Executor()->JointTargetPosition();
    policy_target_position_ = hold_position_;
    if (target_pos_filter_) {
      target_pos_filter_->Reset();
    }
    inference_net_->PrintInfo();
    std::cout << "[PolicyCtrBeyondMimic] Enter BeyondMimic dance policy"
              << std::endl;
  }

  void Step(bool set_target = true) final {
    if (!ready_) {
      std::cerr << "PolicyCtrBeyondMimic not ready" << std::endl;
      return;
    }

    if (counter_++ % decimation_ == 0) {
      if (!policy_started_ && command_.size() > 0 && command_[0] > 0.15f) {
        policy_started_ = true;
      }

      if (policy_started_) {
        inference_net_->InferUnsync(MakeObservation());
      }
    }

    if (!set_target) {
      return;
    }

    if (!policy_started_) {
      robot_->Executor()->JointTargetPosition() =
          target_pos_filter_ ? target_pos_filter_->Filter(hold_position_)
                             : hold_position_;
      robot_->Executor()->JointTargetPGain() = p_gains_;
      robot_->Executor()->JointTargetDGain() = d_gains_;
      return;
    }

    auto target_pos = inference_net_->GetResult();
    if (target_pos.has_value()) {
      policy_target_position_ = default_position_;
      for (size_t i = 0; i < action_size_; ++i) {
        policy_target_position_[policy_joint_idx_map_.at(i)] =
            target_pos.value()[i];
      }
    }

    robot_->Executor()->JointTargetPosition() =
        target_pos_filter_ ? target_pos_filter_->Filter(policy_target_position_)
                           : policy_target_position_;
    robot_->Executor()->JointTargetPGain() = p_gains_;
    robot_->Executor()->JointTargetDGain() = d_gains_;
  }

  void Stop() final {
    command_ = VectorT::Zero(3);
    ready_ = false;
    policy_started_ = false;
  }

 private:
  RobotObservation<float> MakeObservation() {
    VectorT pos_input = VectorT::Zero(action_size_);
    VectorT vel_input = VectorT::Zero(action_size_);
    for (size_t i = 0; i < action_size_; ++i) {
      const size_t joint_idx = policy_joint_idx_map_.at(i);
      pos_input[i] = robot_->Observer()->JointActualPosition()[joint_idx];
      vel_input[i] = robot_->Observer()->JointActualVelocity()[joint_idx];
    }

    return {.command = command_,
            .ang_vel = robot_->Observer()->AngularVelocity(),
            .proj_gravity = robot_->Observer()->ProjGravity(),
            .joint_pos = pos_input,
            .joint_vel = vel_input,
            .euler_angles = robot_->Observer()->EulerRpy()};
  }

 private:
  bool ready_ = false;
  bool auto_start_ = false;
  bool policy_started_ = false;
  size_t action_size_ = 0;
  std::map<size_t, size_t> policy_joint_idx_map_;
  VectorT hold_position_;
};

}  // namespace ovinf

#endif  // POLICY_CTR_BEYOND_MIMIC_HPP

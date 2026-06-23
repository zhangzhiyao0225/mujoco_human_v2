#ifndef POLICY_CTR_AUTO_HPP
#define POLICY_CTR_AUTO_HPP

#include <algorithm>
#include <chrono>

#include "filter/filter_mean.hpp"
#include "policy_controller_base.hpp"

namespace ovinf {

class PolicyCtrAuto : public PolicyControllerBase {
 public:
  using Ptr = std::shared_ptr<PolicyCtrAuto>;

  PolicyCtrAuto() = delete;
  ~PolicyCtrAuto() = default;

  PolicyCtrAuto(RobotBase<float>::RobotPtr robot, YAML::Node const& config)
      : PolicyControllerBase(robot, config) {
    action_size_ = config["inference"]["action_size"].as<size_t>();
    startup_blend_time_ = config["startup_blend_time"].as<double>(0.0);

    for (size_t i = 0; i < action_size_; i++) {
      std::string joint_name =
          config["inference"]["policy_joint_names"][i].as<std::string>();
      size_t joint_idx = robot_->joint_names_.at(joint_name);
      policy_joint_idx_map_[i] = joint_idx;
    }
  }

  virtual void WarmUp() final {
    if (counter_++ % decimation_ == 0) {
      VectorT pos_input = VectorT::Zero(action_size_);
      VectorT vel_input = VectorT::Zero(action_size_);
      for (size_t i = 0; i < action_size_; i++) {
        pos_input[i] =
            robot_->Observer()->JointActualPosition()[policy_joint_idx_map_[i]];
        vel_input[i] =
            robot_->Observer()->JointActualVelocity()[policy_joint_idx_map_[i]];
      }

      inference_net_->WarmUp({.command = command_,
                              .ang_vel = robot_->Observer()->AngularVelocity(),
                              .proj_gravity = robot_->Observer()->ProjGravity(),
                              .joint_pos = pos_input,
                              .joint_vel = vel_input,
                              .euler_angles = robot_->Observer()->EulerRpy()});
    }
    policy_target_position_ = robot_->Executor()->JointTargetPosition();
    target_pos_filter_->Filter(policy_target_position_);
    // target_pos_filter_->Filter(policy_target_position_);
    // inference_net_->PrintInfo();
  }

  virtual void Init() final {
    counter_ = 0;
    command_ = VectorT::Zero(3);
    ready_ = true;
    target_pos_filter_->Reset();
    policy_target_position_ = robot_->Executor()->JointTargetPosition();
    startup_blend_pending_ = startup_blend_time_ > 1e-6;
    startup_blending_ = false;
  }

  virtual void Step(bool set_target = true) final {
    if (!ready_) {
      std::cerr << "PolicyController not ready" << std::endl;
      return;
    }

    if (counter_++ % decimation_ == 0) {
      VectorT pos_input = VectorT::Zero(action_size_);
      VectorT vel_input = VectorT::Zero(action_size_);
      for (size_t i = 0; i < action_size_; i++) {
        pos_input[i] =
            robot_->Observer()->JointActualPosition()[policy_joint_idx_map_[i]];
        vel_input[i] =
            robot_->Observer()->JointActualVelocity()[policy_joint_idx_map_[i]];
      }

      auto err = inference_net_->InferUnsync(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos = pos_input,
           .joint_vel = vel_input,
           .euler_angles = robot_->Observer()->EulerRpy()});
    }

    if (set_target) {
      auto target_pos = inference_net_->GetResult();
      if (target_pos.has_value()) {
        policy_target_position_ = default_position_;
        for (size_t i = 0; i < action_size_; i++) {
          policy_target_position_[policy_joint_idx_map_[i]] =
              target_pos.value()[i];
        }
        policy_target_position_ = ApplyStartupBlend(policy_target_position_);
      } else {
        // std::cout << "target pos is empty" << std::endl;
      }

      robot_->Executor()->JointTargetPosition() =
          target_pos_filter_->Filter(policy_target_position_);
      robot_->Executor()->JointTargetPGain() = p_gains_;
      robot_->Executor()->JointTargetDGain() = d_gains_;
    }
  }

  virtual void Stop() final {
    command_ = VectorT::Zero(3);
    ready_ = false;
    startup_blend_pending_ = false;
    startup_blending_ = false;
  }

 private:
  static double SmoothStep(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  }

  VectorT ApplyStartupBlend(const VectorT& desired_target) {
    if (startup_blend_pending_) {
      startup_blend_pending_ = false;
      startup_blending_ = true;
      startup_blend_start_time_ = std::chrono::steady_clock::now();
      startup_blend_start_target_ = robot_->Executor()->JointTargetPosition();
      std::cout << "[PolicyCtrAuto] Startup blend: " << startup_blend_time_
                << "s" << std::endl;
    }

    if (!startup_blending_) {
      return desired_target;
    }

    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      startup_blend_start_time_)
            .count();
    const double alpha = SmoothStep(elapsed / startup_blend_time_);
    if (alpha >= 1.0) {
      startup_blending_ = false;
      return desired_target;
    }
    return (1.0 - alpha) * startup_blend_start_target_ +
           alpha * desired_target;
  }

  size_t action_size_ = 0;
  std::map<size_t, size_t> policy_joint_idx_map_;
  double startup_blend_time_ = 0.0;
  bool startup_blend_pending_ = false;
  bool startup_blending_ = false;
  VectorT startup_blend_start_target_;
  std::chrono::steady_clock::time_point startup_blend_start_time_;
};

}  // namespace ovinf

#endif  // !POLICY_CTR_AUTO_HPP

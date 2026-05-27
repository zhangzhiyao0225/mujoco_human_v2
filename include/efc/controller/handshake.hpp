#ifndef HANDSHAKE_HPP
#define HANDSHAKE_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

#include "policy_controller_base.hpp"

namespace ovinf {

class HandshakeController : public PolicyControllerBase {
 public:
  using Ptr = std::shared_ptr<HandshakeController>;

  HandshakeController() = delete;
  ~HandshakeController() = default;

  HandshakeController(RobotBase<float>::RobotPtr robot,
                      YAML::Node const& policy_config,
                      YAML::Node const& handshake_config)
      : PolicyControllerBase(robot, policy_config) {
    start_position_ = VectorT::Zero(robot_->joint_size_);
    InitHandshakeConfig(handshake_config);
  }

  void InitHandshakeConfig(YAML::Node const& config) {
    reach_duration_ = ReadFloat(config, "reach_duration", reach_duration_);
    hold_duration_ = ReadFloat(config, "hold_duration", hold_duration_);
    return_duration_ = ReadFloat(config, "return_duration", return_duration_);
    shake_frequency_ = ReadFloat(config, "shake_frequency", shake_frequency_);
    shake_amplitude_ = ReadFloat(config, "shake_amplitude", shake_amplitude_);

    shoulder_pitch_offset_ =
        ReadFloat(config, "shoulder_pitch_offset", shoulder_pitch_offset_);
    shoulder_roll_offset_ =
        ReadFloat(config, "shoulder_roll_offset", shoulder_roll_offset_);
    shoulder_yaw_offset_ =
        ReadFloat(config, "shoulder_yaw_offset", shoulder_yaw_offset_);
    elbow_pitch_offset_ =
        ReadFloat(config, "elbow_pitch_offset", elbow_pitch_offset_);
    elbow_yaw_offset_ = ReadFloat(config, "elbow_yaw_offset", elbow_yaw_offset_);
  }

  virtual void WarmUp() final {
    if (counter_++ % decimation_ == 0) {
      inference_net_->WarmUp(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos = robot_->Observer()->JointActualPosition().segment(0, 13),
           .joint_vel = robot_->Observer()->JointActualVelocity().segment(0, 13)});
    }
    policy_target_position_ = robot_->Executor()->JointTargetPosition();
    target_pos_filter_->Filter(policy_target_position_);
  }

  virtual void Init() final {
    counter_ = 0;
    command_ = VectorT::Zero(3);
    start_position_ = robot_->Observer()->JointActualPosition();
    policy_target_position_ = start_position_;
    start_time_ = std::chrono::high_resolution_clock::now();
    target_pos_filter_->Reset();
    ready_ = true;
    std::cout << "[HandshakeController] Enter handshake state" << std::endl;
  }

  virtual void Step(bool set_target = true) final {
    if (!ready_) {
      std::cerr << "HandshakeController not ready" << std::endl;
      return;
    }

    if (counter_++ % decimation_ == 0) {
      inference_net_->InferUnsync(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos = robot_->Observer()->JointActualPosition().segment(0, 13),
           .joint_vel = robot_->Observer()->JointActualVelocity().segment(0, 13)});
    }

    const float elapsed =
        std::chrono::duration<float>(std::chrono::high_resolution_clock::now() -
                                     start_time_)
            .count();
    const float total_duration =
        std::max(reach_duration_ + hold_duration_ + return_duration_, 1e-4f);

    VectorT handshake_pose = robot_->Executor()->JointTargetPosition();
    auto target_pos = inference_net_->GetResult();
    if (target_pos.has_value()) {
      for (size_t i = 0; i < 13; i++) {
        handshake_pose[i] = target_pos.value()[i];
      }
    } else {
      handshake_pose = policy_target_position_;
    }

    for (size_t i = 13; i < static_cast<size_t>(handshake_pose.size()); i++) {
      handshake_pose[i] = default_position_[i];
    }
    ApplyHandshakePose(handshake_pose, elapsed);

    if (elapsed < reach_duration_) {
      const float ratio = SmoothStep(elapsed / std::max(reach_duration_, 1e-4f));
      policy_target_position_ = handshake_pose;
      for (size_t i = 13; i < static_cast<size_t>(policy_target_position_.size()); i++) {
        policy_target_position_[i] = start_position_[i] +
                                     ratio * (handshake_pose[i] - start_position_[i]);
      }
    } else if (elapsed < reach_duration_ + hold_duration_) {
      policy_target_position_ = handshake_pose;
    } else if (elapsed < total_duration) {
      const float t = elapsed - reach_duration_ - hold_duration_;
      const float ratio = SmoothStep(t / std::max(return_duration_, 1e-4f));
      policy_target_position_ = handshake_pose;
      for (size_t i = 13; i < static_cast<size_t>(policy_target_position_.size()); i++) {
        policy_target_position_[i] = handshake_pose[i] +
                                     ratio * (default_position_[i] - handshake_pose[i]);
      }
    } else {
      policy_target_position_ = handshake_pose;
      for (size_t i = 13; i < static_cast<size_t>(policy_target_position_.size()); i++) {
        policy_target_position_[i] = default_position_[i];
      }
    }

    if (set_target) {
      robot_->Executor()->JointTargetPosition() =
          target_pos_filter_->Filter(policy_target_position_);
      robot_->Executor()->JointTargetPGain() = p_gains_;
      robot_->Executor()->JointTargetDGain() = d_gains_;
    }
  }

  virtual void Stop() final {
    command_ = VectorT::Zero(3);
    ready_ = false;
  }

 private:
  float ReadFloat(YAML::Node const& config, const std::string& key,
                  float default_value) const {
    return config[key] ? config[key].as<float>() : default_value;
  }

  float SmoothStep(float x) const {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * x * (x * (x * 6.0f - 15.0f) + 10.0f);
  }

  int JointIndex(const std::string& name) const {
    const auto it = robot_->joint_names_.find(name);
    return it == robot_->joint_names_.end() ? -1 : static_cast<int>(it->second);
  }

  void SetJoint(VectorT& target, const std::string& name, float value) const {
    const int idx = JointIndex(name);
    if (idx >= 0 && idx < target.size()) {
      target[idx] = value;
    }
  }

  float JointDefault(const std::string& name) const {
    const int idx = JointIndex(name);
    if (idx >= 0 && idx < default_position_.size()) {
      return default_position_[idx];
    }
    return 0.0f;
  }

  void ApplyHandshakePose(VectorT& target, float elapsed) const {
    const float hold_t = std::max(0.0f, elapsed - reach_duration_);
    const bool in_hold =
        elapsed >= reach_duration_ && elapsed < reach_duration_ + hold_duration_;
    const float shake =
        in_hold ? shake_amplitude_ *
                      std::sin(2.0f * static_cast<float>(M_PI) *
                               shake_frequency_ * hold_t)
                : 0.0f;

    SetJoint(target, "r_shoulder_p",
             JointDefault("r_shoulder_p") + shoulder_pitch_offset_);
    SetJoint(target, "r_shoulder_r",
             JointDefault("r_shoulder_r") + shoulder_roll_offset_);
    SetJoint(target, "r_shoulder_y",
             JointDefault("r_shoulder_y") + shoulder_yaw_offset_);
    SetJoint(target, "r_elbow_p",
             JointDefault("r_elbow_p") + elbow_pitch_offset_ + shake);
    SetJoint(target, "r_elbow_y",
             JointDefault("r_elbow_y") + elbow_yaw_offset_);
  }

 private:
  VectorT start_position_;
  std::chrono::high_resolution_clock::time_point start_time_;

  float reach_duration_ = 2.0f;
  float hold_duration_ = 4.0f;
  float return_duration_ = 2.0f;
  float shake_frequency_ = 1.0f;
  float shake_amplitude_ = 0.05f;

  float shoulder_pitch_offset_ = -0.85f;
  float shoulder_roll_offset_ = -0.18f;
  float shoulder_yaw_offset_ = 0.20f;
  float elbow_pitch_offset_ = -0.15f;
  float elbow_yaw_offset_ = -0.45f;
};

}  // namespace ovinf

#endif  // HANDSHAKE_HPP

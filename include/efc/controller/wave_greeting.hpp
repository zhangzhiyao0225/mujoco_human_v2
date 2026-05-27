#ifndef WAVE_GREETING_HPP
#define WAVE_GREETING_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

#include "policy_controller_base.hpp"

namespace ovinf {

class WaveGreetingController : public PolicyControllerBase {
 public:
  using Ptr = std::shared_ptr<WaveGreetingController>;

  WaveGreetingController() = delete;
  ~WaveGreetingController() = default;

  WaveGreetingController(RobotBase<float>::RobotPtr robot,
                         YAML::Node const& policy_config,
                         YAML::Node const& wave_config)
      : PolicyControllerBase(robot, policy_config) {
    start_position_ = VectorT::Zero(robot_->joint_size_);
    InitWaveConfig(wave_config);
  }

  void InitWaveConfig(YAML::Node const& config) {
    raise_duration_ = ReadFloat(config, "raise_duration", raise_duration_);
    wave_duration_ = ReadFloat(config, "wave_duration", wave_duration_);
    lower_duration_ = ReadFloat(config, "lower_duration", lower_duration_);
    wave_frequency_ = ReadFloat(config, "wave_frequency", wave_frequency_);

    shoulder_pitch_offset_ =
        ReadFloat(config, "shoulder_pitch_offset", shoulder_pitch_offset_);
    shoulder_roll_offset_ =
        ReadFloat(config, "shoulder_roll_offset", shoulder_roll_offset_);
    elbow_pitch_offset_ =
        ReadFloat(config, "elbow_pitch_offset", elbow_pitch_offset_);
    shoulder_yaw_amplitude_ =
        ReadFloat(config, "shoulder_yaw_amplitude", shoulder_yaw_amplitude_);
    elbow_yaw_offset_ = ReadFloat(config, "elbow_yaw_offset", elbow_yaw_offset_);
    elbow_yaw_amplitude_ =
        ReadFloat(config, "elbow_yaw_amplitude", elbow_yaw_amplitude_);
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
    std::cout << "[WaveGreetingController] Enter wave greeting state" << std::endl;
  }

  virtual void Step(bool set_target = true) final {
    if (!ready_) {
      std::cerr << "WaveGreetingController not ready" << std::endl;
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
        std::max(raise_duration_ + wave_duration_ + lower_duration_, 1e-4f);

    VectorT wave_pose = robot_->Executor()->JointTargetPosition();
    auto target_pos = inference_net_->GetResult();
    if (target_pos.has_value()) {
      for (size_t i = 0; i < 13; i++) {
        wave_pose[i] = target_pos.value()[i];
      }
    } else {
      wave_pose = policy_target_position_;
    }

    for (size_t i = 13; i < static_cast<size_t>(wave_pose.size()); i++) {
      wave_pose[i] = default_position_[i];
    }
    ApplyRaisedArmPose(wave_pose, elapsed);

    if (elapsed < raise_duration_) {
      const float ratio = SmoothStep(elapsed / std::max(raise_duration_, 1e-4f));
      policy_target_position_ = wave_pose;
      for (size_t i = 13; i < static_cast<size_t>(policy_target_position_.size()); i++) {
        policy_target_position_[i] =
            start_position_[i] + ratio * (wave_pose[i] - start_position_[i]);
      }
    } else if (elapsed < raise_duration_ + wave_duration_) {
      policy_target_position_ = wave_pose;
    } else if (elapsed < total_duration) {
      const float t = elapsed - raise_duration_ - wave_duration_;
      const float ratio = SmoothStep(t / std::max(lower_duration_, 1e-4f));
      policy_target_position_ = wave_pose;
      for (size_t i = 13; i < static_cast<size_t>(policy_target_position_.size()); i++) {
        policy_target_position_[i] =
            wave_pose[i] + ratio * (default_position_[i] - wave_pose[i]);
      }
    } else {
      policy_target_position_ = wave_pose;
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

  void ApplyRaisedArmPose(VectorT& target, float elapsed) const {
    const float wave_t = std::max(0.0f, elapsed - raise_duration_);
    const float wave_gate =
        SmoothStep(std::min(wave_t, 0.35f) / 0.35f) *
        SmoothStep(std::min(raise_duration_ + wave_duration_ - elapsed, 0.35f) /
                   0.35f);
    const float phase =
        2.0f * static_cast<float>(M_PI) * wave_frequency_ * wave_t;

    SetJoint(target, "r_shoulder_p",
             JointDefault("r_shoulder_p") + shoulder_pitch_offset_);
    SetJoint(target, "r_shoulder_r",
             JointDefault("r_shoulder_r") + shoulder_roll_offset_);
    SetJoint(target, "r_elbow_p",
             JointDefault("r_elbow_p") + elbow_pitch_offset_);
    SetJoint(target, "r_shoulder_y",
             JointDefault("r_shoulder_y") +
                 wave_gate * shoulder_yaw_amplitude_ * std::sin(phase));
    SetJoint(target, "r_elbow_y",
             JointDefault("r_elbow_y") + elbow_yaw_offset_ +
                 wave_gate * elbow_yaw_amplitude_ * std::sin(phase));
  }

 private:
  VectorT start_position_;
  std::chrono::high_resolution_clock::time_point start_time_;

  float raise_duration_ = 1.8f;
  float wave_duration_ = 4.0f;
  float lower_duration_ = 2.2f;
  float wave_frequency_ = 1.0f;

  float shoulder_pitch_offset_ = -0.50f;
  float shoulder_roll_offset_ = -0.20f;
  float elbow_pitch_offset_ = -2.8f;
  float shoulder_yaw_amplitude_ = 0.16f;
  float elbow_yaw_offset_ = 0.90f;
  float elbow_yaw_amplitude_ = -0.10f;
};

}  // namespace ovinf

#endif  // WAVE_GREETING_HPP

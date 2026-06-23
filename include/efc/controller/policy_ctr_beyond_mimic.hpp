#ifndef POLICY_CTR_BEYOND_MIMIC_HPP
#define POLICY_CTR_BEYOND_MIMIC_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <limits>
#include <string>
#include <vector>

#include "ovinf/ovinf_beyond_mimic.h"
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
    startup_blend_time_ =
        config["inference"]["startup_blend_time"].as<double>(0.0);
    finish_margin_ = config["inference"]["finish_margin"].as<size_t>(2);
    if (config["safety_clamp"]) {
      knee_min_ = config["safety_clamp"]["knee_min"].as<double>(
          -std::numeric_limits<double>::infinity());
      knee_max_ = config["safety_clamp"]["knee_max"].as<double>(
          std::numeric_limits<double>::infinity());
      waist_y_min_ = config["safety_clamp"]["waist_y_min"].as<double>(
          -std::numeric_limits<double>::infinity());
      waist_y_max_ = config["safety_clamp"]["waist_y_max"].as<double>(
          std::numeric_limits<double>::infinity());
      hip_roll_abs_ = config["safety_clamp"]["hip_roll_abs"].as<double>(
          std::numeric_limits<double>::infinity());
      hip_yaw_abs_ = config["safety_clamp"]["hip_yaw_abs"].as<double>(
          std::numeric_limits<double>::infinity());
      ankle_pitch_min_ = config["safety_clamp"]["ankle_pitch_min"].as<double>(
          -std::numeric_limits<double>::infinity());
      ankle_pitch_max_ = config["safety_clamp"]["ankle_pitch_max"].as<double>(
          std::numeric_limits<double>::infinity());
      ankle_roll_abs_ = config["safety_clamp"]["ankle_roll_abs"].as<double>(
          std::numeric_limits<double>::infinity());
    }

    for (size_t i = 0; i < action_size_; ++i) {
      const std::string joint_name =
          config["inference"]["policy_joint_names"][i].as<std::string>();
      const size_t joint_idx = robot_->joint_names_.at(joint_name);
      policy_joint_idx_map_[i] = joint_idx;
      if (joint_name == "l_knee" || joint_name == "r_knee") {
        knee_joint_indices_.push_back(joint_idx);
      } else if (joint_name == "waist_y") {
        waist_y_joint_indices_.push_back(joint_idx);
      } else if (joint_name == "l_hip_r" || joint_name == "r_hip_r") {
        hip_roll_joint_indices_.push_back(joint_idx);
      } else if (joint_name == "l_hip_y" || joint_name == "r_hip_y") {
        hip_yaw_joint_indices_.push_back(joint_idx);
      } else if (joint_name == "l_ankle_p" || joint_name == "r_ankle_p") {
        ankle_pitch_joint_indices_.push_back(joint_idx);
      } else if (joint_name == "l_ankle_r" || joint_name == "r_ankle_r") {
        ankle_roll_joint_indices_.push_back(joint_idx);
      }
    }

    if (std::isfinite(knee_min_) || std::isfinite(knee_max_) ||
        std::isfinite(waist_y_min_) || std::isfinite(waist_y_max_) ||
        std::isfinite(hip_roll_abs_) || std::isfinite(hip_yaw_abs_) ||
        std::isfinite(ankle_pitch_min_) ||
        std::isfinite(ankle_pitch_max_) || std::isfinite(ankle_roll_abs_)) {
      std::cout << "[PolicyCtrBeyondMimic] Safety clamp: knee=["
                << knee_min_ << ", " << knee_max_ << "], waist_y=["
                << waist_y_min_ << ", " << waist_y_max_
                << "], hip_roll_abs=" << hip_roll_abs_
                << ", hip_yaw_abs=" << hip_yaw_abs_ << ", ankle_pitch=["
                << ankle_pitch_min_ << ", " << ankle_pitch_max_
                << "], ankle_roll_abs=" << ankle_roll_abs_
                << std::endl;
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
    policy_started_ = false;
    startup_blending_ = false;
    startup_first_frame_requested_ = false;
    startup_first_frame_ready_ = false;
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
      if (!policy_started_ &&
          (auto_start_ || (command_.size() > 0 && command_[0] > 0.15f))) {
        StartPolicyBlend();
      }

      if (policy_started_) {
        if (startup_blending_) {
          // 启动接管阶段只请求第 0 帧一次，不继续推进 timestep。
          // 这样站立策略目标可以平滑接到跳舞第 0 帧，避免切换瞬间猛跳。
          if (!startup_first_frame_requested_) {
            inference_net_->InferUnsync(MakeObservation());
            startup_first_frame_requested_ = true;
          }
        } else {
          inference_net_->InferUnsync(MakeObservation());
        }
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

    if (startup_blending_) {
      UpdateStartupBlendTarget();
      robot_->Executor()->JointTargetPosition() =
          target_pos_filter_ ? target_pos_filter_->Filter(policy_target_position_)
                             : policy_target_position_;
      robot_->Executor()->JointTargetPGain() = p_gains_;
      robot_->Executor()->JointTargetDGain() = d_gains_;
      return;
    }

    auto target_pos = inference_net_->GetResult();
    if (target_pos.has_value()) {
      policy_target_position_ = BuildFullTarget(target_pos.value());
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
    startup_blending_ = false;
    startup_first_frame_requested_ = false;
    startup_first_frame_ready_ = false;
  }

  bool IsTrajectoryFinished(size_t margin = std::numeric_limits<size_t>::max()) const {
    auto beyond_mimic =
        std::dynamic_pointer_cast<ovinf::BeyondMimicPolicy>(inference_net_);
    if (!beyond_mimic) {
      return false;
    }
    const size_t effective_margin =
        margin == std::numeric_limits<size_t>::max() ? finish_margin_ : margin;
    return beyond_mimic->IsTrajectoryFinished(effective_margin);
  }

 private:
  void StartPolicyBlend() {
    policy_started_ = true;
    startup_blending_ = startup_blend_time_ > 1e-6;
    startup_first_frame_requested_ = false;
    startup_first_frame_ready_ = false;
    startup_blend_start_target_ = robot_->Executor()->JointTargetPosition();
    hold_position_ = startup_blend_start_target_;
    policy_target_position_ = startup_blend_start_target_;

    if (!startup_blending_) {
      std::cout << "[PolicyCtrBeyondMimic] Start dance without startup blend"
                << std::endl;
      return;
    }

    std::cout << "[PolicyCtrBeyondMimic] Start dance with "
              << startup_blend_time_ << "s startup blend" << std::endl;
  }

  VectorT BuildFullTarget(const VectorT& policy_target) const {
    VectorT full_target = default_position_;
    for (size_t i = 0; i < action_size_; ++i) {
      full_target[policy_joint_idx_map_.at(i)] = policy_target[i];
    }
    ApplySafetyClamp(full_target);
    return full_target;
  }

  void ApplySafetyClamp(VectorT& full_target) const {
    if (std::isfinite(knee_min_) || std::isfinite(knee_max_)) {
      for (const size_t joint_idx : knee_joint_indices_) {
        full_target[joint_idx] = std::clamp(
            full_target[joint_idx], static_cast<float>(knee_min_),
            static_cast<float>(knee_max_));
      }
    }

    if (std::isfinite(waist_y_min_) || std::isfinite(waist_y_max_)) {
      for (const size_t joint_idx : waist_y_joint_indices_) {
        full_target[joint_idx] = std::clamp(
            full_target[joint_idx], static_cast<float>(waist_y_min_),
            static_cast<float>(waist_y_max_));
      }
    }

    if (std::isfinite(hip_roll_abs_)) {
      const float limit = static_cast<float>(hip_roll_abs_);
      for (const size_t joint_idx : hip_roll_joint_indices_) {
        full_target[joint_idx] =
            std::clamp(full_target[joint_idx], -limit, limit);
      }
    }

    if (std::isfinite(hip_yaw_abs_)) {
      const float limit = static_cast<float>(hip_yaw_abs_);
      for (const size_t joint_idx : hip_yaw_joint_indices_) {
        full_target[joint_idx] =
            std::clamp(full_target[joint_idx], -limit, limit);
      }
    }

    if (std::isfinite(ankle_pitch_min_) || std::isfinite(ankle_pitch_max_)) {
      for (const size_t joint_idx : ankle_pitch_joint_indices_) {
        full_target[joint_idx] = std::clamp(
            full_target[joint_idx], static_cast<float>(ankle_pitch_min_),
            static_cast<float>(ankle_pitch_max_));
      }
    }

    if (std::isfinite(ankle_roll_abs_)) {
      const float limit = static_cast<float>(ankle_roll_abs_);
      for (const size_t joint_idx : ankle_roll_joint_indices_) {
        full_target[joint_idx] =
            std::clamp(full_target[joint_idx], -limit, limit);
      }
    }
  }

  static double SmoothStep(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  }

  void UpdateStartupBlendTarget() {
    if (!startup_first_frame_requested_) {
      policy_target_position_ = startup_blend_start_target_;
      return;
    }

    if (!startup_first_frame_ready_) {
      auto target_pos = inference_net_->GetResult();
      if (!target_pos.has_value()) {
        policy_target_position_ = startup_blend_start_target_;
        return;
      }
      startup_blend_end_target_ = BuildFullTarget(target_pos.value());
      startup_blend_start_time_ = std::chrono::steady_clock::now();
      startup_first_frame_ready_ = true;
    }

    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      startup_blend_start_time_)
            .count();
    const double alpha = SmoothStep(elapsed / startup_blend_time_);
    policy_target_position_ =
        (1.0 - alpha) * startup_blend_start_target_ +
        alpha * startup_blend_end_target_;

    if (alpha >= 1.0) {
      startup_blending_ = false;
      policy_target_position_ = startup_blend_end_target_;
      std::cout << "[PolicyCtrBeyondMimic] Startup blend finished"
                << std::endl;
    }
  }

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
  bool startup_blending_ = false;
  bool startup_first_frame_requested_ = false;
  bool startup_first_frame_ready_ = false;
  double startup_blend_time_ = 0.0;
  size_t finish_margin_ = 2;
  double knee_min_ = -std::numeric_limits<double>::infinity();
  double knee_max_ = std::numeric_limits<double>::infinity();
  double waist_y_min_ = -std::numeric_limits<double>::infinity();
  double waist_y_max_ = std::numeric_limits<double>::infinity();
  double hip_roll_abs_ = std::numeric_limits<double>::infinity();
  double hip_yaw_abs_ = std::numeric_limits<double>::infinity();
  double ankle_pitch_min_ = -std::numeric_limits<double>::infinity();
  double ankle_pitch_max_ = std::numeric_limits<double>::infinity();
  double ankle_roll_abs_ = std::numeric_limits<double>::infinity();
  size_t action_size_ = 0;
  std::map<size_t, size_t> policy_joint_idx_map_;
  std::vector<size_t> knee_joint_indices_;
  std::vector<size_t> waist_y_joint_indices_;
  std::vector<size_t> hip_roll_joint_indices_;
  std::vector<size_t> hip_yaw_joint_indices_;
  std::vector<size_t> ankle_pitch_joint_indices_;
  std::vector<size_t> ankle_roll_joint_indices_;
  VectorT hold_position_;
  VectorT startup_blend_start_target_;
  VectorT startup_blend_end_target_;
  std::chrono::steady_clock::time_point startup_blend_start_time_;
};

}  // namespace ovinf

#endif  // POLICY_CTR_BEYOND_MIMIC_HPP

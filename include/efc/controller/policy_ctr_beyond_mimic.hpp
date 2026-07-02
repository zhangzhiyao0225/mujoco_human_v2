#ifndef POLICY_CTR_BEYOND_MIMIC_HPP
#define POLICY_CTR_BEYOND_MIMIC_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "ovinf/ovinf_beyond_mimic.h"
#include "policy_controller_base.hpp"
#include "utils/csv_logger.hpp"

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
    if (config["startup_torque_clamp"]) {
      startup_torque_clamp_time_ =
          config["startup_torque_clamp"]["duration"].as<double>(0.0);
      startup_hip_roll_torque_abs_ =
          config["startup_torque_clamp"]["hip_roll_abs"].as<double>(
              std::numeric_limits<double>::infinity());
      startup_waist_y_torque_abs_ =
          config["startup_torque_clamp"]["waist_y_abs"].as<double>(
              std::numeric_limits<double>::infinity());
      startup_hip_yaw_torque_abs_ =
          config["startup_torque_clamp"]["hip_yaw_abs"].as<double>(
              std::numeric_limits<double>::infinity());
    }
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

    joint_names_by_index_.resize(robot_->joint_size_);
    for (const auto& pair : robot_->joint_names_) {
      joint_names_by_index_[pair.second] = pair.first;
    }

    if (config["inference"]["log_data"].as<uint32_t>() != 0) {
      CreateControllerLog(config["inference"]);
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

    if (startup_torque_clamp_time_ > 1e-6) {
      std::cout << "[PolicyCtrBeyondMimic] Startup torque clamp: duration="
                << startup_torque_clamp_time_
                << "s, hip_roll_abs=" << startup_hip_roll_torque_abs_
                << ", waist_y_abs=" << startup_waist_y_torque_abs_
                << ", hip_yaw_abs=" << startup_hip_yaw_torque_abs_
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
    if (auto beyond_mimic =
            std::dynamic_pointer_cast<ovinf::BeyondMimicPolicy>(inference_net_)) {
      beyond_mimic->ResetTrajectoryState();
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
      WriteControllerLog();
      return;
    }

    if (startup_blending_) {
      UpdateStartupBlendTarget();
      VectorT target = target_pos_filter_ ? target_pos_filter_->Filter(policy_target_position_)
                                           : policy_target_position_;
      ApplyStartupTorqueClamp(target);
      robot_->Executor()->JointTargetPosition() = target;
      robot_->Executor()->JointTargetPGain() = p_gains_;
      robot_->Executor()->JointTargetDGain() = d_gains_;
      WriteControllerLog();
      return;
    }

    auto target_pos = inference_net_->GetResult();
    if (target_pos.has_value()) {
      policy_target_position_ = BuildFullTarget(target_pos.value());
    }

    VectorT target = target_pos_filter_ ? target_pos_filter_->Filter(policy_target_position_)
                                         : policy_target_position_;
    ApplyStartupTorqueClamp(target);
    robot_->Executor()->JointTargetPosition() = target;
    robot_->Executor()->JointTargetPGain() = p_gains_;
    robot_->Executor()->JointTargetDGain() = d_gains_;
    WriteControllerLog();
  }

  void Stop() final {
    command_ = VectorT::Zero(3);
    ready_ = false;
    policy_started_ = false;
    startup_blending_ = false;
    startup_first_frame_requested_ = false;
    startup_first_frame_ready_ = false;
    if (auto beyond_mimic =
            std::dynamic_pointer_cast<ovinf::BeyondMimicPolicy>(inference_net_)) {
      beyond_mimic->ResetTrajectoryState();
    }
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
    policy_start_time_ = std::chrono::steady_clock::now();

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

  bool StartupTorqueClampActive() const {
    if (!policy_started_ || startup_torque_clamp_time_ <= 1e-6) {
      return false;
    }
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      policy_start_time_)
            .count();
    return elapsed < startup_torque_clamp_time_;
  }

  void ClampPdTorqueByTarget(VectorT& target, const std::vector<size_t>& indices,
                             double torque_abs) const {
    if (!std::isfinite(torque_abs)) {
      return;
    }
    const float limit = static_cast<float>(torque_abs);
    const VectorT& q = robot_->Observer()->JointActualPosition();
    const VectorT& qd = robot_->Observer()->JointActualVelocity();
    for (const size_t joint_idx : indices) {
      const float kp = p_gains_[joint_idx];
      if (std::abs(kp) < 1e-6f) {
        continue;
      }
      const float raw_torque =
          kp * (target[joint_idx] - q[joint_idx]) - d_gains_[joint_idx] * qd[joint_idx];
      const float clamped_torque = std::clamp(raw_torque, -limit, limit);
      target[joint_idx] =
          q[joint_idx] + (clamped_torque + d_gains_[joint_idx] * qd[joint_idx]) / kp;
    }
  }

  void ApplyStartupTorqueClamp(VectorT& target) const {
    if (!StartupTorqueClampActive()) {
      return;
    }
    ClampPdTorqueByTarget(target, hip_roll_joint_indices_,
                          startup_hip_roll_torque_abs_);
    ClampPdTorqueByTarget(target, waist_y_joint_indices_,
                          startup_waist_y_torque_abs_);
    ClampPdTorqueByTarget(target, hip_yaw_joint_indices_,
                          startup_hip_yaw_torque_abs_);
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

  void CreateControllerLog(YAML::Node const& config) {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_time);
    std::stringstream ss;
    ss << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S");
    std::string current_time = ss.str();

    std::filesystem::path log_dir(config["log_dir"].as<std::string>());
    if (log_dir.is_relative()) {
      log_dir = std::filesystem::canonical(log_dir);
    }
    if (!std::filesystem::exists(log_dir)) {
      std::filesystem::create_directories(log_dir);
    }

    const std::string log_name = config["log_name"].as<std::string>();
    const std::filesystem::path logger_file =
        log_dir / (current_time + "_humanoid_" + log_name + "_controller.csv");

    std::vector<std::string> headers;
    headers.push_back("policy_started");
    headers.push_back("startup_blending");
    headers.push_back("startup_torque_clamp_active");
    headers.push_back("pd_torque_abs_max");
    headers.push_back("applied_torque_abs_max");
    headers.push_back("torque_limit_saturated_count");
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      headers.push_back("pd_torque_" + joint_names_by_index_[i]);
    }
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      headers.push_back("applied_torque_" + joint_names_by_index_[i]);
    }
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      headers.push_back("joint_target_pos_" + joint_names_by_index_[i]);
    }
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      headers.push_back("joint_actual_pos_" + joint_names_by_index_[i]);
    }
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      headers.push_back("joint_pos_error_" + joint_names_by_index_[i]);
    }

    controller_csv_logger_ =
        std::make_shared<CsvLogger>(logger_file.string(), headers);
  }

  void WriteControllerLog() {
    if (!controller_csv_logger_) {
      return;
    }

    const VectorT& target_pos = robot_->Executor()->JointTargetPosition();
    const VectorT& actual_pos = robot_->Observer()->JointActualPosition();
    const VectorT& actual_vel = robot_->Observer()->JointActualVelocity();
    const VectorT pos_error = target_pos - actual_pos;
    const VectorT pd_torque =
        p_gains_.cwiseProduct(pos_error) - d_gains_.cwiseProduct(actual_vel);

    VectorT applied_torque = pd_torque;
    size_t saturated_count = 0;
    const VectorT& torque_limit = robot_->Executor()->TorqueLimit();
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      const float limit = std::abs(torque_limit[i]);
      if (std::isfinite(limit) && limit > 0.0f) {
        const float clamped = std::clamp(applied_torque[i], -limit, limit);
        if (std::abs(clamped - applied_torque[i]) > 1.0e-5f) {
          ++saturated_count;
        }
        applied_torque[i] = clamped;
      }
    }

    std::vector<CsvLogger::Number> datas;
    datas.push_back(policy_started_ ? 1.0 : 0.0);
    datas.push_back(startup_blending_ ? 1.0 : 0.0);
    datas.push_back(StartupTorqueClampActive() ? 1.0 : 0.0);
    datas.push_back(pd_torque.cwiseAbs().maxCoeff());
    datas.push_back(applied_torque.cwiseAbs().maxCoeff());
    datas.push_back(static_cast<double>(saturated_count));
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      datas.push_back(pd_torque[i]);
    }
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      datas.push_back(applied_torque[i]);
    }
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      datas.push_back(target_pos[i]);
    }
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      datas.push_back(actual_pos[i]);
    }
    for (size_t i = 0; i < robot_->joint_size_; ++i) {
      datas.push_back(pos_error[i]);
    }

    controller_csv_logger_->Write(datas);
  }

 private:
  bool ready_ = false;
  bool auto_start_ = false;
  bool policy_started_ = false;
  bool startup_blending_ = false;
  bool startup_first_frame_requested_ = false;
  bool startup_first_frame_ready_ = false;
  double startup_blend_time_ = 0.0;
  double startup_torque_clamp_time_ = 0.0;
  double startup_hip_roll_torque_abs_ = std::numeric_limits<double>::infinity();
  double startup_waist_y_torque_abs_ = std::numeric_limits<double>::infinity();
  double startup_hip_yaw_torque_abs_ = std::numeric_limits<double>::infinity();
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
  std::vector<std::string> joint_names_by_index_;
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
  std::chrono::steady_clock::time_point policy_start_time_;
  CsvLogger::Ptr controller_csv_logger_;
};

}  // namespace ovinf

#endif  // POLICY_CTR_BEYOND_MIMIC_HPP

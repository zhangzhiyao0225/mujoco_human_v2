#ifndef HEAD_SURVEY_HPP
#define HEAD_SURVEY_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

#include "policy_controller_base.hpp"

namespace ovinf {

class HeadSurveyController : public PolicyControllerBase {
 public:
  using Ptr = std::shared_ptr<HeadSurveyController>;

  HeadSurveyController() = delete;
  ~HeadSurveyController() = default;

  HeadSurveyController(RobotBase<float>::RobotPtr robot,
                       YAML::Node const& policy_config,
                       YAML::Node const& survey_config)
      : PolicyControllerBase(robot, policy_config) {
    start_position_ = VectorT::Zero(robot_->joint_size_);
    InitSurveyConfig(survey_config);
  }

  void InitSurveyConfig(YAML::Node const& config) {
    turn_left_duration_ =
        ReadFloat(config, "turn_left_duration", turn_left_duration_);
    turn_right_duration_ =
        ReadFloat(config, "turn_right_duration", turn_right_duration_);
    return_duration_ = ReadFloat(config, "return_duration", return_duration_);
    left_yaw_ = ReadFloat(config, "left_yaw", left_yaw_);
    right_yaw_ = ReadFloat(config, "right_yaw", right_yaw_);
    center_yaw_ = ReadFloat(config, "center_yaw", center_yaw_);
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
    start_head_yaw_ = JointActual("head_y");
    policy_target_position_ = start_position_;
    start_time_ = std::chrono::high_resolution_clock::now();
    target_pos_filter_->Reset();
    ready_ = true;
    std::cout << "[HeadSurveyController] Enter head survey state" << std::endl;
  }

  virtual void Step(bool set_target = true) final {
    if (!ready_) {
      std::cerr << "HeadSurveyController not ready" << std::endl;
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
        std::max(turn_left_duration_ + turn_right_duration_ + return_duration_,
                 1e-4f);

    VectorT survey_pose = robot_->Executor()->JointTargetPosition();
    auto target_pos = inference_net_->GetResult();
    if (target_pos.has_value()) {
      for (size_t i = 0; i < 13; i++) {
        survey_pose[i] = target_pos.value()[i];
      }
    } else {
      survey_pose = policy_target_position_;
    }

    for (size_t i = 13; i < static_cast<size_t>(survey_pose.size()); i++) {
      survey_pose[i] = default_position_[i];
    }

    if (elapsed < turn_left_duration_) {
      const float ratio =
          SmoothStep(elapsed / std::max(turn_left_duration_, 1e-4f));
      policy_target_position_ = survey_pose;
      for (size_t i = 13; i < static_cast<size_t>(policy_target_position_.size());
           i++) {
        policy_target_position_[i] =
            start_position_[i] + ratio * (survey_pose[i] - start_position_[i]);
      }
      SetJoint(policy_target_position_, "head_y",
               start_head_yaw_ + ratio * (left_yaw_ - start_head_yaw_));
    } else if (elapsed < turn_left_duration_ + turn_right_duration_) {
      const float t = elapsed - turn_left_duration_;
      const float ratio =
          SmoothStep(t / std::max(turn_right_duration_, 1e-4f));
      policy_target_position_ = survey_pose;
      SetJoint(policy_target_position_, "head_y",
               left_yaw_ + ratio * (right_yaw_ - left_yaw_));
    } else if (elapsed < total_duration) {
      const float t = elapsed - turn_left_duration_ - turn_right_duration_;
      const float ratio = SmoothStep(t / std::max(return_duration_, 1e-4f));
      policy_target_position_ = survey_pose;
      SetJoint(policy_target_position_, "head_y",
               right_yaw_ + ratio * (center_yaw_ - right_yaw_));
    } else {
      policy_target_position_ = survey_pose;
      SetJoint(policy_target_position_, "head_y", center_yaw_);
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

  float JointActual(const std::string& name) const {
    const int idx = JointIndex(name);
    if (idx >= 0 && idx < start_position_.size()) {
      return start_position_[idx];
    }
    return 0.0f;
  }

 private:
  VectorT start_position_;
  std::chrono::high_resolution_clock::time_point start_time_;

  float turn_left_duration_ = 2.5f;
  float turn_right_duration_ = 4.0f;
  float return_duration_ = 2.5f;

  float left_yaw_ = 0.78539816f;
  float right_yaw_ = -0.78539816f;
  float center_yaw_ = 0.0f;
  float start_head_yaw_ = 0.0f;
};

}  // namespace ovinf

#endif  // HEAD_SURVEY_HPP

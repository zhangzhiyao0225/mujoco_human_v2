#include "ovinf/ovinf_beyond_mimic.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ovinf {

BeyondMimicPolicy::~BeyondMimicPolicy() {
  exiting_.store(true);
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

BeyondMimicPolicy::BeyondMimicPolicy(const YAML::Node &config)
    : BasePolicy(config) {
  // Read config
  size_t joint_counter = 0;
  for (auto const &name : config["policy_joint_names"]) {
    joint_names_[name.as<std::string>()] = joint_counter++;
  }

  single_obs_size_ = config["single_obs_size"].as<size_t>();
  current_obs_ = VectorT(single_obs_size_).setZero();
  action_size_ = config["action_size"].as<size_t>();
  if (action_size_ != joint_counter) {
    throw std::runtime_error("Action size mismatch");
  }
  action_scale_ = config["action_scale"].as<float>();
  obs_scale_ang_vel_ = config["obs_scales"]["ang_vel"].as<float>();
  obs_scale_command_ = config["obs_scales"]["command"].as<float>();
  obs_scale_dof_pos_ = config["obs_scales"]["dof_pos"].as<float>();
  obs_scale_dof_vel_ = config["obs_scales"]["dof_vel"].as<float>();
  clip_action_ = config["clip_action"].as<float>();
  auto_start_ = config["auto_start"].as<bool>(false);
  startup_timestep_offset_ = config["startup_timestep_offset"].as<size_t>(0);
  trajectory_time_scale_ = config["trajectory_time_scale"].as<float>(1.0f);
  startup_match_mode_ = config["startup_match_mode"].as<std::string>("offset");
  startup_match_stride_ = config["startup_match_stride"].as<size_t>(2);
  startup_match_min_timestep_ = config["startup_match_min_timestep"].as<size_t>(0);
  startup_match_max_timestep_ = config["startup_match_max_timestep"].as<size_t>(std::numeric_limits<size_t>::max());
  startup_match_joint_weight_ = config["startup_match_joint_weight"].as<float>(1.0f);
  startup_match_orientation_weight_ =
      config["startup_match_orientation_weight"].as<float>(0.0f);
  joint_default_position_ = VectorT(joint_names_.size());
  stick_to_core_ = config["stick_to_core"].as<size_t>();
  log_name_ = config["log_name"].as<std::string>();

  for (auto const &pair : joint_names_) {
    joint_default_position_(pair.second, 0) =
        config["policy_default_position"][pair.first].as<float>();
  }

  // Create buffer
  last_action_ = VectorT(action_size_).setZero();
  latest_target_ = joint_default_position_;

  // Create logger
  log_flag_ = config["log_data"].as<uint32_t>() ? true : false;
  if (log_flag_) {
    CreateLog(config);
  }

  // Create model
  auto model = ov::Core().read_model(model_path_);
  // compiled_model_ = ov::Core().compile_model(model_path_, device_);
  compiled_model_ = ov::Core().compile_model(model, device_);
  if (compiled_model_.input(0).get_element_type() != ov::element::f32) {
    throw std::runtime_error(
        "Model input type is not f32. Please convert the model to f32.");
  }

  // Get Trajectory Info
  dancing_started_.store(false);
  if (config["traj_length"]) {
    traj_length_ = config["traj_length"].as<size_t>();
  }
  try {
    auto rt_info = model->get_rt_info();
    auto direct = rt_info.find("traj_length");
    if (direct != rt_info.end()) {
      traj_length_ = direct->second.as<size_t>();
    }
    for (const auto &info : rt_info) {
      try {
        for (const auto &pair : info.second.as<ov::AnyMap>()) {
          if (pair.first == "traj_length") {
            traj_length_ = pair.second.as<size_t>();
          }
        }
      } catch (...) {
      }
    }
  } catch (...) {
  }
  if (traj_length_ == 0) {
    traj_length_ = std::numeric_limits<size_t>::max();
  }

  infer_request_ = compiled_model_.create_infer_request();
  obs_info_ = compiled_model_.input(0);
  timestep_info_ = compiled_model_.input(1);

  ref_joint_pos_ = joint_default_position_;
  ref_joint_vel_ = VectorT::Zero(action_size_);
  ref_base_quat_ = QuaternionT::Identity();

  inference_done_.store(true);
  exiting_.store(false);
  worker_thread_ = std::thread(&BeyondMimicPolicy::WorkerThread, this);
}

void BeyondMimicPolicy::ResetTrajectoryState() {
  motion_phase_input_ = static_cast<float>(startup_timestep_offset_);
  timestep_input_ = std::floor(motion_phase_input_);
  dancing_started_.store(false);
  corrected_bias_flag_.store(false);
  last_action_ = VectorT(action_size_).setZero();
  latest_target_ = joint_default_position_;
  ref_joint_pos_ = joint_default_position_;
  ref_joint_vel_ = VectorT::Zero(action_size_);
  ref_base_quat_ = QuaternionT::Identity();
  yaw_bias_ = QuaternionT::Identity();
  init_q_yaw_ = QuaternionT::Identity();
}

bool BeyondMimicPolicy::WarmUp(RobotObservation<float> const &obs_pack) {
  VectorT obs(single_obs_size_);
  obs.setZero();
  timestep_input_ = 0.0;
  motion_phase_input_ = 0.0f;
  dancing_started_.store(false);
  last_action_ = VectorT(action_size_).setZero();
  latest_target_ = joint_default_position_;
  ref_joint_pos_ = joint_default_position_;
  ref_joint_vel_ = VectorT::Zero(action_size_);
  ref_base_quat_ = QuaternionT::Identity();

  if (!inference_done_.load()) {
    return false;
  } else {
    current_obs_ = obs;

    ov::Tensor obs_tensor(obs_info_.get_element_type(), obs_info_.get_shape(),
                          current_obs_.data());
    ov::Tensor timestep_tensor(timestep_info_.get_element_type(),
                               timestep_info_.get_shape(), &timestep_input_);

    // Get yaw bias
    init_q_yaw_ = AngleAxisT(obs_pack.euler_angles(2), Vector3T::UnitZ());
    Vector3T first_ref_ori_rpy =
        ref_base_quat_.toRotationMatrix().eulerAngles(0, 1, 2);
    yaw_bias_ =
        (init_q_yaw_ * AngleAxisT(-first_ref_ori_rpy(2), Vector3T::UnitZ()))
            .normalized();

    infer_start_time_ = std::chrono::high_resolution_clock::now();

    infer_request_.set_input_tensor(0, obs_tensor);
    infer_request_.set_input_tensor(1, timestep_tensor);
    inference_done_.store(false);

    return true;
  }
}

bool BeyondMimicPolicy::InferUnsync(RobotObservation<float> const &obs_pack) {
  VectorT obs(single_obs_size_);
  obs.setZero();

  if ((auto_start_ || obs_pack.command(0, 0) > 0.15f) &&
      !dancing_started_.load()) {
    dancing_started_.store(true);
    const size_t startup_timestep = SelectStartupTimestep(obs_pack);
    motion_phase_input_ = static_cast<float>(startup_timestep);
    timestep_input_ = std::floor(motion_phase_input_);
  } else if (dancing_started_.load() &&
             (traj_length_ == std::numeric_limits<size_t>::max() ||
              timestep_input_ < static_cast<float>(traj_length_ - 1))) {
    motion_phase_input_ += trajectory_time_scale_;
    timestep_input_ = std::floor(motion_phase_input_);
  } else if (dancing_started_.load() &&
             traj_length_ != std::numeric_limits<size_t>::max()) {
    motion_phase_input_ = static_cast<float>(traj_length_ - 1);
    timestep_input_ = static_cast<float>(traj_length_ - 1);
  } else {
    motion_phase_input_ = static_cast<float>(startup_timestep_offset_);
    timestep_input_ = std::floor(motion_phase_input_);

    // Get yaw bias
    init_q_yaw_ = AngleAxisT(obs_pack.euler_angles(2), Vector3T::UnitZ());
    Vector3T first_ref_ori_rpy =
        ref_base_quat_.toRotationMatrix().eulerAngles(0, 1, 2);
    yaw_bias_ =
        (init_q_yaw_ * AngleAxisT(-first_ref_ori_rpy(2), Vector3T::UnitZ()))
            .normalized();

    dancing_started_.store(false);
  }

  // Yaw correction
  ref_base_quat_ = yaw_bias_ * ref_base_quat_;

  // Orientation difference
  QuaternionT current_anchor_orientation =
      AngleAxisT(obs_pack.euler_angles(2), Vector3T::UnitZ()) *
      AngleAxisT(obs_pack.euler_angles(1), Vector3T::UnitY()) *
      AngleAxisT(obs_pack.euler_angles(0), Vector3T::UnitX());
  Matrix3T orientation_diff_matrix =
      (current_anchor_orientation.inverse() * ref_base_quat_)
          .toRotationMatrix();
  VectorT ori_input(6);
  ori_input(0, 0) = orientation_diff_matrix(0, 0);
  ori_input(1, 0) = orientation_diff_matrix(0, 1);
  ori_input(2, 0) = orientation_diff_matrix(1, 0);
  ori_input(3, 0) = orientation_diff_matrix(1, 1);
  ori_input(4, 0) = orientation_diff_matrix(2, 0);
  ori_input(5, 0) = orientation_diff_matrix(2, 1);

  // Reference joint pos
  obs.segment(0, action_size_) = ref_joint_pos_;
  // Refenence joint vel
  obs.segment(action_size_, action_size_) = ref_joint_vel_;
  obs.segment(2 * action_size_, 6) = ori_input;
  obs.segment(6 + 2 * action_size_, 3) = obs_pack.ang_vel;
  obs.segment(9 + 2 * action_size_, action_size_) =
      obs_pack.joint_pos - joint_default_position_;
  obs.segment(9 + 3 * action_size_, action_size_) = obs_pack.joint_vel;
  obs.segment(9 + 4 * action_size_, action_size_) = last_action_;

  if (!inference_done_.load()) {
    return false;
  } else {
    current_obs_ = obs;
    ov::Tensor obs_tensor(obs_info_.get_element_type(), obs_info_.get_shape(),
                          current_obs_.data());
    ov::Tensor timestep_tensor(timestep_info_.get_element_type(),
                               timestep_info_.get_shape(), &timestep_input_);
    infer_start_time_ = std::chrono::high_resolution_clock::now();

    infer_request_.set_input_tensor(0, obs_tensor);
    infer_request_.set_input_tensor(1, timestep_tensor);
    inference_done_.store(false);

    if (log_flag_) {
      WriteLog(obs_pack);
    }

    return true;
  }
}


size_t BeyondMimicPolicy::SelectStartupTimestep(
    RobotObservation<float> const &obs_pack) {
  if (startup_match_mode_ != "joint_nearest") {
    return startup_timestep_offset_;
  }
  if (!inference_done_.load()) {
    return startup_timestep_offset_;
  }

  const size_t effective_traj_length =
      traj_length_ == std::numeric_limits<size_t>::max() ? 0 : traj_length_;
  if (effective_traj_length == 0) {
    return startup_timestep_offset_;
  }

  VectorT zero_obs(single_obs_size_);
  zero_obs.setZero();
  ov::Tensor obs_tensor(obs_info_.get_element_type(), obs_info_.get_shape(),
                        zero_obs.data());

  const size_t stride = std::max<size_t>(1, startup_match_stride_);
  float best_score = std::numeric_limits<float>::infinity();
  size_t best_timestep = startup_timestep_offset_;
  VectorT best_ref_joint_pos = ref_joint_pos_;
  VectorT best_ref_joint_vel = ref_joint_vel_;
  QuaternionT best_ref_quat = ref_base_quat_;

  QuaternionT current_orientation =
      AngleAxisT(obs_pack.euler_angles(2), Vector3T::UnitZ()) *
      AngleAxisT(obs_pack.euler_angles(1), Vector3T::UnitY()) *
      AngleAxisT(obs_pack.euler_angles(0), Vector3T::UnitX());

  const size_t search_start = std::min(startup_match_min_timestep_, effective_traj_length - 1);
  const size_t search_end = std::min(startup_match_max_timestep_, effective_traj_length - 1);
  for (size_t t = search_start; t <= search_end; t += stride) {
    float timestep = static_cast<float>(t);
    ov::Tensor timestep_tensor(timestep_info_.get_element_type(),
                               timestep_info_.get_shape(), &timestep);
    infer_request_.set_input_tensor(0, obs_tensor);
    infer_request_.set_input_tensor(1, timestep_tensor);
    infer_request_.infer();

    auto ref_joint_pos_tensor = infer_request_.get_output_tensor(1);
    auto ref_joint_vel_tensor = infer_request_.get_output_tensor(2);
    auto body_quat_tensor = infer_request_.get_output_tensor(4);
    VectorT ref_joint_pos =
        Eigen::Map<VectorT>(ref_joint_pos_tensor.data<float>(), action_size_);

    const float joint_error =
        (obs_pack.joint_pos - ref_joint_pos).squaredNorm() /
        static_cast<float>(std::max<size_t>(1, action_size_));

    const float* body_quat_data = body_quat_tensor.data<float>() + 4 * anchor_body_index_;
    QuaternionT ref_quat(body_quat_data[0], body_quat_data[1],
                         body_quat_data[2], body_quat_data[3]);
    const Matrix3T orientation_diff =
        (current_orientation.inverse() * ref_quat).toRotationMatrix();
    const float orientation_error =
        (3.0f - orientation_diff.trace()) * 0.5f;

    const float score = startup_match_joint_weight_ * joint_error +
                        startup_match_orientation_weight_ * orientation_error;
    if (score < best_score) {
      best_score = score;
      best_timestep = t;
      best_ref_joint_pos = ref_joint_pos;
      best_ref_joint_vel =
          Eigen::Map<VectorT>(ref_joint_vel_tensor.data<float>(), action_size_);
      best_ref_quat = ref_quat;
    }
  }

  ref_joint_pos_ = best_ref_joint_pos;
  ref_joint_vel_ = best_ref_joint_vel;
  ref_base_quat_ = best_ref_quat;

  Vector3T first_ref_ori_rpy =
      ref_base_quat_.toRotationMatrix().eulerAngles(0, 1, 2);
  init_q_yaw_ = AngleAxisT(obs_pack.euler_angles(2), Vector3T::UnitZ());
  yaw_bias_ =
      (init_q_yaw_ * AngleAxisT(-first_ref_ori_rpy(2), Vector3T::UnitZ()))
          .normalized();

  std::cout << "[BeyondMimicPolicy] startup matched timestep "
            << best_timestep << " score=" << best_score << std::endl;
  return best_timestep;
}

std::optional<BeyondMimicPolicy::VectorT> BeyondMimicPolicy::GetResult(
    const size_t timeout) {
  if (inference_done_.load()) {
    return latest_target_;
  } else {
    std::this_thread::sleep_for(std::chrono::microseconds(timeout));
    if (inference_done_.load()) {
      return latest_target_;
    }
    return std::nullopt;
  }
}

void BeyondMimicPolicy::PrintInfo() {
  std::cout << "Load model: " << this->model_path_ << std::endl;
  std::cout << "Device: " << device_ << std::endl;
  std::cout << "Single obs size: " << single_obs_size_ << std::endl;
  std::cout << "Action size: " << action_size_ << std::endl;
  std::cout << "Action scale: " << action_scale_ << std::endl;
  std::cout << "Startup timestep offset: " << startup_timestep_offset_
            << std::endl;
  std::cout << "Startup match mode: " << startup_match_mode_
            << ", stride: " << startup_match_stride_
            << ", range: [" << startup_match_min_timestep_ << ", "
            << startup_match_max_timestep_ << "]"
            << ", joint weight: " << startup_match_joint_weight_
            << ", orientation weight: " << startup_match_orientation_weight_
            << std::endl;
  std::cout << "Trajectory time scale: " << trajectory_time_scale_
            << std::endl;
  std::cout << "Anchor body index: " << anchor_body_index_ << std::endl;
  std::cout << "  - Obs scale ang vel: " << obs_scale_ang_vel_ << std::endl;
  std::cout << "  - Obs scale command: " << obs_scale_command_ << std::endl;
  std::cout << "  - Obs scale dof pos: " << obs_scale_dof_pos_ << std::endl;
  std::cout << "  - Obs scale dof vel: " << obs_scale_dof_vel_ << std::endl;
  std::cout << "Clip action: " << clip_action_ << std::endl;
  std::cout << "Joint default position: " << joint_default_position_.transpose()
            << std::endl;
  std::cout << "Joint names: " << std::endl;
  for (const auto &pair : joint_names_) {
    std::cout << "  - " << pair.first << ": " << pair.second << std::endl;
  }
}

bool BeyondMimicPolicy::IsTrajectoryFinished(size_t margin) const {
  if (traj_length_ == std::numeric_limits<size_t>::max() || traj_length_ <= margin + 1) {
    return false;
  }
  if (!dancing_started_.load()) {
    return false;
  }
  return timestep_input_ >= static_cast<float>(traj_length_ - 1 - margin);
}

void BeyondMimicPolicy::WorkerThread() {
  if (realtime_) {
    if (!setProcessHighPriority(99)) {
      std::cerr << "Failed to set process high priority." << std::endl;
    }
    if (!StickThisThreadToCore(stick_to_core_)) {
      std::cerr << "Failed to stick thread to core." << std::endl;
    }
  }

  while (exiting_.load() == false) {
    if (!inference_done_.load()) {
      infer_request_.infer();
      auto action_tensor = infer_request_.get_output_tensor(0);
      auto ref_joint_pos_tensor = infer_request_.get_output_tensor(1);
      auto ref_joint_vel_tensor = infer_request_.get_output_tensor(2);
      auto body_quat_tensor = infer_request_.get_output_tensor(4);

      infer_end_time_ = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_seconds =
          infer_end_time_ - infer_start_time_;
      // std::cout << "Inference time: " << elapsed_seconds.count() * 1000
      //           << "ms" << std::endl;
      inference_time_ = elapsed_seconds.count() * 1000;

      ref_joint_pos_ =
          Eigen::Map<VectorT>(ref_joint_pos_tensor.data<float>(), action_size_);
      ref_joint_vel_ =
          Eigen::Map<VectorT>(ref_joint_vel_tensor.data<float>(), action_size_);
      const float* body_quat_data = body_quat_tensor.data<float>() + 4 * anchor_body_index_;
      ref_base_quat_ = QuaternionT(body_quat_data[0], body_quat_data[1],
                                   body_quat_data[2], body_quat_data[3]);

      VectorT action_eigen =
          Eigen::Map<VectorT>(action_tensor.data<float>(), action_size_)
              .cwiseMin(clip_action_)
              .cwiseMax(-clip_action_);

      last_action_ = action_eigen;
      latest_target_ = action_eigen * action_scale_ + joint_default_position_;
    }
    inference_done_.store(true);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
}

void BeyondMimicPolicy::CreateLog(YAML::Node const &config) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm *now_tm = std::localtime(&now_time);
  std::stringstream ss;
  ss << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S");
  std::string current_time = ss.str();

  std::string log_dir = config["log_dir"].as<std::string>();
  std::filesystem::path config_file_path(log_dir);
  if (config_file_path.is_relative()) {
    config_file_path = canonical(config_file_path);
  }

  if (!exists(config_file_path)) {
    create_directories(config_file_path);
  }

  std::string logger_file = config_file_path.string() + "/" + current_time +
                            "_humanoid_" + log_name_ + ".csv";

  // Get headers
  std::vector<std::string> headers;

  headers.push_back("command_vel_x");
  headers.push_back("command_vel_y");
  headers.push_back("command_vel_w");
  for (size_t i = 0; i < action_size_; ++i) {
    headers.push_back("joint_pos_" + std::to_string(i));
  }
  for (size_t i = 0; i < action_size_; ++i) {
    headers.push_back("joint_vel_" + std::to_string(i));
  }
  for (size_t i = 0; i < action_size_; ++i) {
    headers.push_back("last_action_" + std::to_string(i));
  }
  headers.push_back("ang_vel_x");
  headers.push_back("ang_vel_y");
  headers.push_back("ang_vel_z");
  headers.push_back("prog_gravity_x");
  headers.push_back("prog_gravity_y");
  headers.push_back("prog_gravity_z");
  headers.push_back("timestep");
  headers.push_back("inference_time_ms");

  csv_logger_ = std::make_shared<CsvLogger>(logger_file, headers);
}

void BeyondMimicPolicy::WriteLog(RobotObservation<float> const &obs_pack) {
  std::vector<CsvLogger::Number> datas;

  for (size_t i = 0; i < 3; ++i) {
    datas.push_back(obs_pack.command(i));
  }
  for (size_t i = 0; i < action_size_; ++i) {
    datas.push_back(obs_pack.joint_pos(i));
  }
  for (size_t i = 0; i < action_size_; ++i) {
    datas.push_back(obs_pack.joint_vel(i));
  }
  for (size_t i = 0; i < action_size_; ++i) {
    datas.push_back(last_action_(i));
  }
  for (size_t i = 0; i < 3; ++i) {
    datas.push_back(obs_pack.ang_vel(i));
  }
  for (size_t i = 0; i < 3; ++i) {
    datas.push_back(obs_pack.proj_gravity(i));
  }
  datas.push_back(timestep_input_);
  datas.push_back(inference_time_);

  csv_logger_->Write(datas);
}

}  // namespace ovinf

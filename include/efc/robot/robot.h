#ifndef ROBOT_EFC_HPP
#define ROBOT_EFC_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <mutex>
#include <thread>
#include <vector>

#include "filter/filter_factory.hpp"
#include "robot/base/robot_base.hpp"
#include "robot/common.h"
#include "utils/csv_logger.hpp"
#include "utils/csv_dds_logger.hpp"

namespace ovinf
{

  class RobotEfc : public RobotBase<float>
  {
    using VectorT = Eigen::Matrix<float, Eigen::Dynamic, 1>;
    friend class ObserverEfc;
    friend class ExecutorEfc;

  public:
    using Ptr = std::shared_ptr<RobotEfc>;

  private:
    class ObserverEfc : public ObserverBase
    {
    public:
      ObserverEfc() = delete;
      ObserverEfc(RobotBase<float> *robot, const YAML::Node &config)
          : ObserverBase(robot, config)
      {
        // Create Filter
        motor_pos_filter_ =
            FilterFactory::CreateFilter(config["motor_pos_filter"]);
        motor_vel_filter_ =
            FilterFactory::CreateFilter(config["motor_vel_filter"]);
        ang_vel_filter_ = FilterFactory::CreateFilter(config["ang_vel_filter"]);
        acc_filter_ = FilterFactory::CreateFilter(config["acc_filter"]);
        eluer_rpy_filter_ = FilterFactory::CreateFilter(config["euler_filter"]);

        // Create Logger
        log_flag_ = config["log_data"].as<uint32_t>();

        std::string log_dir = config["log_dir"].as<std::string>();
        std::filesystem::path config_file_path(log_dir);
        if (config_file_path.is_relative())
        {
          config_file_path = canonical(config_file_path);
        }

        if (!std::filesystem::exists(config_file_path))
        {
          std::filesystem::create_directories(config_file_path);
        }

        log_file_path_.swap(config_file_path);

        CreateCsvLog();
      }

      virtual bool Update() final
      {
        auto robot_efc = dynamic_cast<RobotEfc *>(robot_);

        for (size_t i = 0; i < motor_size_; ++i)
        {
          motor_actual_position_[i] = robot_efc->motors_[i]->GetActualPosition() *
                                      robot_->motor_direction_(i, 0);
          motor_actual_velocity_[i] = robot_efc->motors_[i]->GetActualVelocity() *
                                      robot_->motor_direction_(i, 0);

          motor_motor_temp_[i] = robot_efc->motors_[i]->GetMotorTemperature();
          motor_mos_temp_[i] = robot_efc->motors_[i]->GetMosTemperature();
        }

        motor_actual_position_ =
            motor_pos_filter_->Filter(motor_actual_position_);
        motor_actual_velocity_ =
            motor_vel_filter_->Filter(motor_actual_velocity_);

        joint_actual_position_ = motor_actual_position_;
        joint_actual_velocity_ = motor_actual_velocity_;

        // MuJoCo exposes ankle pitch/roll directly. Real hardware can enable
        // this to convert between serial joint and parallel motor coordinates.
        if (robot_efc->use_parallel_ankle_)
        {
          auto left_joint_pos = robot_efc->ankles_[LEFT]->ForwardKinematics(
              motor_actual_position_[LAnkleShortMotor],
              motor_actual_position_[LAnkleLongMotor]);
          auto right_joint_pos = robot_efc->ankles_[RIGHT]->ForwardKinematics(
              motor_actual_position_[RAnkleShortMotor],
              motor_actual_position_[RAnkleLongMotor]);
          auto left_joint_vel = robot_efc->ankles_[LEFT]->VelocityP2S(
              motor_actual_velocity_[LAnkleShortMotor],
              motor_actual_velocity_[LAnkleLongMotor]);
          auto right_joint_vel = robot_efc->ankles_[RIGHT]->VelocityP2S(
              motor_actual_velocity_[RAnkleShortMotor],
              motor_actual_velocity_[RAnkleLongMotor]);

          joint_actual_position_[LAnklePitchJoint] = left_joint_pos(0, 0);
          joint_actual_position_[LAnkleRollJoint] = left_joint_pos(1, 0);
          joint_actual_position_[RAnklePitchJoint] = right_joint_pos(0, 0);
          joint_actual_position_[RAnkleRollJoint] = right_joint_pos(1, 0);

          joint_actual_velocity_[LAnklePitchJoint] = left_joint_vel(0, 0);
          joint_actual_velocity_[LAnkleRollJoint] = left_joint_vel(1, 0);
          joint_actual_velocity_[RAnklePitchJoint] = right_joint_vel(0, 0);
          joint_actual_velocity_[RAnkleRollJoint] = right_joint_vel(1, 0);

          // Set frontend extra data
          robot_efc->extra_data_->Set<"LAnklePitchPos">(
              joint_actual_position_[LAnklePitchJoint] / M_PI * 180.0);
          robot_efc->extra_data_->Set<"LAnkleRollPos">(
              joint_actual_position_[LAnkleRollJoint] / M_PI * 180.0);
          robot_efc->extra_data_->Set<"RAnklePitchPos">(
              joint_actual_position_[RAnklePitchJoint] / M_PI * 180.0);
          robot_efc->extra_data_->Set<"RAnkleRollPos">(
              joint_actual_position_[RAnkleRollJoint] / M_PI * 180.0);

          robot_efc->extra_data_->Set<"LAnklePitchVel">(
              joint_actual_velocity_[LAnklePitchJoint] / M_PI * 180.0);
          robot_efc->extra_data_->Set<"LAnkleRollVel">(
              joint_actual_velocity_[LAnkleRollJoint] / M_PI * 180.0);
          robot_efc->extra_data_->Set<"RAnklePitchVel">(
              joint_actual_velocity_[RAnklePitchJoint] / M_PI * 180.0);
          robot_efc->extra_data_->Set<"RAnkleRollVel">(
              joint_actual_velocity_[RAnkleRollJoint] / M_PI * 180.0);
        }

        euler_rpy_ = eluer_rpy_filter_->Filter(
            (VectorT(3) << robot_efc->imu_->GetRoll(),
             robot_efc->imu_->GetPitch(), robot_efc->imu_->GetYaw())
                .finished());

        acceleration_ = acc_filter_->Filter(
            (VectorT(3) << robot_efc->imu_->GetAccX(), robot_efc->imu_->GetAccY(),
             robot_efc->imu_->GetAccZ())
                .finished());

        angular_velocity_ = ang_vel_filter_->Filter(
            (VectorT(3) << robot_efc->imu_->GetGyroX(),
             robot_efc->imu_->GetGyroY(), robot_efc->imu_->GetGyroZ())
                .finished());

        Eigen::Matrix3f Rwb(
            Eigen::AngleAxisf(euler_rpy_[2], Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(euler_rpy_[1], Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(euler_rpy_[0], Eigen::Vector3f::UnitX()));
        proj_gravity_ =
            VectorT(Rwb.transpose() * Eigen::Vector3f{0.0, 0.0, -1.0});

        if (log_flag_)
        {
          WriteLog();
        }
        return true;
      }

    private:
      void GenerateLogHeader(std::vector<std::string> &headers)
      {
        headers.clear();

        // Motor actual pos
        for (size_t i = 0; i < motor_size_; ++i)
        {
          headers.push_back("motor_actual_pos_" + std::to_string(i));
        }

        // Motor actual vel
        for (size_t i = 0; i < motor_size_; ++i)
        {
          headers.push_back("motor_actual_vel_" + std::to_string(i));
        }

        // Joint actual pos
        for (size_t i = 0; i < joint_size_; ++i)
        {
          headers.push_back("joint_actual_pos_" + std::to_string(i));
        }

        // Joint actual vel
        for (size_t i = 0; i < joint_size_; ++i)
        {
          headers.push_back("joint_actual_vel_" + std::to_string(i));
        }

        // Motor target pos
        for (size_t i = 0; i < motor_size_; ++i)
        {
          headers.push_back("motor_target_pos_" + std::to_string(i));
        }

        // Motor target torque
        for (size_t i = 0; i < motor_size_; ++i)
        {
          headers.push_back("motor_target_torque_" + std::to_string(i));
        }

        // Joint target pos
        for (size_t i = 0; i < joint_size_; ++i)
        {
          headers.push_back("joint_target_pos_" + std::to_string(i));
        }

        // Joint target torque
        for (size_t i = 0; i < joint_size_; ++i)
        {
          headers.push_back("joint_target_torque_" + std::to_string(i));
        }

        // Motor motor temp
        for (size_t i = 0; i < motor_size_; ++i)
        {
          headers.push_back("motor_temp_" + std::to_string(i));
        }

        // Motor motor temp
        for (size_t i = 0; i < motor_size_; ++i)
        {
          headers.push_back("mos_temp_" + std::to_string(i));
        }

        // Acc
        headers.push_back("acc_x");
        headers.push_back("acc_y");
        headers.push_back("acc_z");

        // Ang vel
        headers.push_back("ang_vel_x");
        headers.push_back("ang_vel_y");
        headers.push_back("ang_vel_z");

        // Euler RPY
        headers.push_back("euler_roll");
        headers.push_back("euler_pitch");
        headers.push_back("euler_yaw");

        // Proj gravity
        headers.push_back("proj_gravity_x");
        headers.push_back("proj_gravity_y");
        headers.push_back("proj_gravity_z");
      }

    public:
      inline void CreateCsvDds();

    private:
      inline void CreateCsvLog();
      inline void WriteLog();

    private:
      FilterBase<VectorT>::Ptr motor_pos_filter_;
      FilterBase<VectorT>::Ptr motor_vel_filter_;
      FilterBase<VectorT>::Ptr ang_vel_filter_;
      FilterBase<VectorT>::Ptr acc_filter_;
      FilterBase<VectorT>::Ptr eluer_rpy_filter_;

      uint32_t log_flag_ = 0;
      CsvLogger::Ptr csv_logger_;
      CsvDdsLogger::Ptr dds_logger_;
      std::filesystem::path log_file_path_;
    };

    class ExecutorEfc : public ExecutorBase
    {
    public:
      ExecutorEfc() = delete;
      ExecutorEfc(RobotBase<float> *robot, const YAML::Node &config)
          : ExecutorBase(robot, config) {}

      virtual bool ExecuteJointTorque() final
      {
        // Deprecated
        std::cerr << "ExecuteJointTorque is deprecated." << std::endl;
        motor_target_position_ = joint_target_position_;
        motor_target_torque_ = joint_target_torque_;

        ExecuteMotorTorque();
        return true;
      }

      virtual bool ExecuteJointPosition() final
      {
        auto robot_efc = dynamic_cast<RobotEfc *>(robot_);

        motor_target_position_ = joint_target_position_;
        motor_target_torque_.setZero();
        motor_target_p_gain_ = joint_target_p_gain_;
        motor_target_d_gain_ = joint_target_d_gain_;

        // Left ankle
        if (robot_efc->use_parallel_ankle_)
        {
          auto output = robot_efc->ankles_[LEFT]->PDTargetS2P(
              {.serial_kp = {joint_target_p_gain_[LAnklePitchJoint],
                             joint_target_p_gain_[LAnkleRollJoint]},
               .serial_kd =
                   {
                       joint_target_d_gain_[LAnklePitchJoint],
                       joint_target_d_gain_[LAnkleRollJoint],
                   },
               .serial_q =
                   {
                       robot_->Observer()
                           ->JointActualPosition()[LAnklePitchJoint],
                       robot_->Observer()->JointActualPosition()[LAnkleRollJoint],
                   },
               .serial_target_q =
                   {
                       joint_target_position_[LAnklePitchJoint],
                       joint_target_position_[LAnkleRollJoint],
                   },
               .parallel_q =
                   {
                       robot_->Observer()
                           ->MotorActualPosition()[LAnkleShortMotor],
                       robot_->Observer()->MotorActualPosition()[LAnkleLongMotor],
                   },
               .parallel_dq = {
                   robot_->Observer()->MotorActualVelocity()[LAnkleShortMotor],
                   robot_->Observer()->MotorActualVelocity()[LAnkleLongMotor],
               }});

          motor_target_position_[LAnkleShortMotor] =
              output.parallel_target_pos(0, 0);
          motor_target_position_[LAnkleLongMotor] =
              output.parallel_target_pos(1, 0);

          motor_target_p_gain_[LAnkleShortMotor] = output.parallel_kp(0, 0);
          motor_target_p_gain_[LAnkleLongMotor] = output.parallel_kp(1, 0);

          motor_target_d_gain_[LAnkleShortMotor] = output.parallel_kd(0, 0);
          motor_target_d_gain_[LAnkleLongMotor] = output.parallel_kd(1, 0);

          motor_target_torque_[LAnkleShortMotor] =
              output.feedforward_torque(0, 0);
          motor_target_torque_[LAnkleLongMotor] = output.feedforward_torque(1, 0);
        }

        // Right ankle
        if (robot_efc->use_parallel_ankle_)
        {
          auto output = robot_efc->ankles_[RIGHT]->PDTargetS2P(
              {.serial_kp = {joint_target_p_gain_[RAnklePitchJoint],
                             joint_target_p_gain_[RAnkleRollJoint]},
               .serial_kd =
                   {
                       joint_target_d_gain_[RAnklePitchJoint],
                       joint_target_d_gain_[RAnkleRollJoint],
                   },
               .serial_q =
                   {
                       robot_->Observer()
                           ->JointActualPosition()[RAnklePitchJoint],
                       robot_->Observer()->JointActualPosition()[RAnkleRollJoint],
                   },
               .serial_target_q =
                   {
                       joint_target_position_[RAnklePitchJoint],
                       joint_target_position_[RAnkleRollJoint],
                   },
               .parallel_q =
                   {
                       robot_->Observer()
                           ->MotorActualPosition()[RAnkleShortMotor],
                       robot_->Observer()->MotorActualPosition()[RAnkleLongMotor],
                   },
               .parallel_dq = {
                   robot_->Observer()->MotorActualVelocity()[RAnkleShortMotor],
                   robot_->Observer()->MotorActualVelocity()[RAnkleLongMotor],
               }});

          motor_target_position_[RAnkleShortMotor] =
              output.parallel_target_pos(0, 0);
          motor_target_position_[RAnkleLongMotor] =
              output.parallel_target_pos(1, 0);

          motor_target_p_gain_[RAnkleShortMotor] = output.parallel_kp(0, 0);
          motor_target_p_gain_[RAnkleLongMotor] = output.parallel_kp(1, 0);

          motor_target_d_gain_[RAnkleShortMotor] = output.parallel_kd(0, 0);
          motor_target_d_gain_[RAnkleLongMotor] = output.parallel_kd(1, 0);

          motor_target_torque_[RAnkleShortMotor] =
              output.feedforward_torque(0, 0);
          motor_target_torque_[RAnkleLongMotor] = output.feedforward_torque(1, 0);
        }

        ExecuteMotorPosition();
        return true;
      }

      virtual bool ExecuteMotorPosition() final
      {
        auto robot_efc = dynamic_cast<RobotEfc *>(robot_);

        // Set target
        for (size_t i = 0; i < motor_size_; ++i)
        {
          robot_efc->motors_[i]->SetTargetTorque(motor_target_torque_[i] *
                                                 robot_->motor_direction_(i, 0));
          robot_efc->motors_[i]->SetTargetPosition(
              motor_target_position_[i] * robot_->motor_direction_(i, 0));
          robot_efc->motors_[i]->SetTargetPDGains(motor_target_p_gain_[i],
                                                  motor_target_d_gain_[i]);
        }
        return true;
      }

      virtual bool ExecuteMotorTorque() final
      {
        // Deprecated
        std::cerr << "ExecuteMotorTorque is deprecated." << std::endl;

        auto robot_efc = dynamic_cast<RobotEfc *>(robot_);
        for (size_t i = 0; i < motor_size_; ++i)
        {
          // Torque limit
          if (motor_target_torque_[i] > torque_limit_[i])
          {
            motor_target_torque_[i] = torque_limit_[i];
          }
          else if (motor_target_torque_[i] < -torque_limit_[i])
          {
            motor_target_torque_[i] = -torque_limit_[i];
          }

          // Position limit
          if (robot_->Observer()->MotorActualPosition()[i] >
              motor_upper_limit_[i])
          {
            motor_target_torque_[i] = 0.0;
          }
          else if (robot_->Observer()->MotorActualPosition()[i] <
                   motor_lower_limit_[i])
          {
            motor_target_torque_[i] = 0.0;
          }
        }

        // Set target
        for (size_t i = 0; i < motor_size_; ++i)
        {
          robot_efc->motors_[i]->SetTargetTorque(motor_target_torque_[i] *
                                                 robot_->motor_direction_(i, 0));
          robot_efc->motors_[i]->SetTargetPosition(
              motor_target_position_[i] * robot_->motor_direction_(i, 0));
        }
        return true;
      }

      virtual bool ExecuteMotorCurrent() final
      {
        throw std::runtime_error("ExecuteMotorCurrent is not supported");
        return false;
      }

    private:
    };

  public:
    RobotEfc() = delete;
    RobotEfc(const YAML::Node &config) : RobotBase(config)
    {
      if (config["use_parallel_ankle"])
      {
        use_parallel_ankle_ = config["use_parallel_ankle"].as<bool>();
      }

      motors_.resize(motor_size_);
      this->observer_ = std::make_shared<ObserverEfc>((RobotBase<float> *)this,
                                                      config["observer"]);
      this->executor_ = std::make_shared<ExecutorEfc>((RobotBase<float> *)this,
                                                      config["executor"]);

      // Create ankle resolver from yaml config file
      this->ankles_.resize(2);
      this->ankles_[LEFT] = AnkleFromYaml(config["ankle_left"]);
      this->ankles_[RIGHT] = AnkleFromYaml(config["ankle_right"]);
    }

    AnklePtr AnkleFromYaml(YAML::Node const &config)
    {
      return std::make_shared<AnkleT>(AnkleT::AnkleParameters{
          .l_bar1 = config["l_bar1"].as<float>(),
          .l_rod1 = config["l_rod1"].as<float>(),
          .r_a1 = Yaml2Eigen(config["r_a1"]),
          .r_b1_0 = Yaml2Eigen(config["r_b1_0"]),
          .r_c1_0 = Yaml2Eigen(config["r_c1_0"]),
          .l_bar2 = config["l_bar2"].as<float>(),
          .l_rod2 = config["l_rod2"].as<float>(),
          .r_a2 = Yaml2Eigen(config["r_a2"]),
          .r_b2_0 = Yaml2Eigen(config["r_b2_0"]),
          .r_c2_0 = Yaml2Eigen(config["r_c2_0"]),
          .r_op = Yaml2Eigen(config["r_op"]),
      });
    }

    inline VectorT Yaml2Eigen(YAML::Node const &config)
    {
      return Eigen::Map<VectorT>(config.as<std::vector<float>>().data(),
                                 config.size());
    }
    inline void GetDevice(const KernelBus &bus);

    void SetExtraData(Kernel::ExtraData &extra_data)
    {
      extra_data_ = &extra_data;
    }

    virtual void PrintInfo() final
    {
      for (auto const &pair : motor_names_)
      {
        std::cout << "Motor id: " << pair.second << ", name: " << pair.first
                  << std::endl;
        std::cout << "  - direction: " << motor_direction_(pair.second, 0)
                  << std::endl;
        std::cout << "  - upper limit: "
                  << executor_->MotorUpperLimit()(pair.second, 0) << std::endl;
        std::cout << "  - lower limit: "
                  << executor_->MotorLowerLimit()(pair.second, 0) << std::endl;
        std::cout << "  - torque limit: "
                  << executor_->TorqueLimit()(pair.second, 0) << std::endl;
      }
      for (auto const &pair : joint_names_)
      {
        std::cout << "Joint id: " << pair.second << " name: " << pair.first
                  << std::endl;
      }
    }

    void SetRobNotiPublisher(handop::RobotNotificationPublisher::Ptr noti_pub)
    {
      noti_pub_ = noti_pub;
      dynamic_cast<ObserverEfc *>(this->Observer().get())->CreateCsvDds();
    }

    handop::RobotNotificationPublisher::Ptr GetRobNotiPublisher()
    {
      return noti_pub_;
    }

  private:
    std::vector<MotorPtr> motors_ = {};
    handop::RobotNotificationPublisher::Ptr noti_pub_{};
    ImuPtr imu_;
    Kernel::ExtraData *extra_data_;
    std::vector<AnklePtr> ankles_;
    bool use_parallel_ankle_ = false;
  };

  void RobotEfc::GetDevice(const KernelBus &bus)
  {
    for (size_t i = 0; i < joint_size_; ++i)
    {
      motors_[i] = bus.GetDevice<MotorDevice>(i).value();
    }
    imu_ = bus.GetDevice<ImuDevice>(joint_size_).value();
  }

  void RobotEfc::ObserverEfc::CreateCsvLog()
  {
    if (!(log_flag_ & (LOG_DATA_FLAG_CSV)))
    {
      return;
    }

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm *now_tm = std::localtime(&now_time);
    std::stringstream ss;
    ss << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S");
    std::string current_time = ss.str();
    std::string logger_file =
        log_file_path_.string() + "/" + current_time + "_extra.csv";

    if (!std::filesystem::exists(logger_file))
    {
      std::filesystem::create_directories(logger_file);
    }

    // Get headers
    std::vector<std::string> headers;
    GenerateLogHeader(headers);

    csv_logger_ = std::make_shared<CsvLogger>(logger_file, headers);
  }

  void RobotEfc::ObserverEfc::CreateCsvDds()
  {
    if (!(log_flag_ & LOG_DATA_FLAG_DDS) || dds_logger_ != nullptr)
    {
      return;
    }

    // Get headers
    std::vector<std::string> headers;
    GenerateLogHeader(headers);

    dds_logger_ = std::make_shared<CsvDdsLogger>(dynamic_cast<RobotEfc *>(robot_)->GetRobNotiPublisher(), headers);
  }

  void RobotEfc::ObserverEfc::WriteLog()
  {
    std::vector<CsvLogger::Number> datas;

    // Motor actual pos
    for (size_t i = 0; i < motor_size_; ++i)
    {
      datas.push_back(motor_actual_position_[i]);
    }

    // Motor actual vel
    for (size_t i = 0; i < motor_size_; ++i)
    {
      datas.push_back(motor_actual_velocity_[i]);
    }

    // Joint actual pos
    for (size_t i = 0; i < joint_size_; ++i)
    {
      datas.push_back(joint_actual_position_[i]);
    }

    // Joint actual vel
    for (size_t i = 0; i < joint_size_; ++i)
    {
      datas.push_back(joint_actual_velocity_[i]);
    }

    // Motor target pos
    for (size_t i = 0; i < motor_size_; ++i)
    {
      datas.push_back(robot_->Executor()->MotorTargetPosition()[i]);
    }

    // Motor target torque
    for (size_t i = 0; i < motor_size_; ++i)
    {
      datas.push_back(robot_->Executor()->MotorTargetTorque()[i]);
    }

    // Joint target pos
    for (size_t i = 0; i < joint_size_; ++i)
    {
      datas.push_back(robot_->Executor()->JointTargetPosition()[i]);
    }

    // Joint target torque
    for (size_t i = 0; i < joint_size_; ++i)
    {
      datas.push_back(robot_->Executor()->JointTargetTorque()[i]);
    }

    // Motor motor temp
    for (size_t i = 0; i < motor_size_; ++i)
    {
      datas.push_back(motor_motor_temp_[i]);
    }

    // Motor motor temp
    for (size_t i = 0; i < motor_size_; ++i)
    {
      datas.push_back(motor_mos_temp_[i]);
    }

    // Acc
    datas.push_back(acceleration_[0]);
    datas.push_back(acceleration_[1]);
    datas.push_back(acceleration_[2]);

    // Ang vel
    datas.push_back(angular_velocity_[0]);
    datas.push_back(angular_velocity_[1]);
    datas.push_back(angular_velocity_[2]);

    // Euler RPY
    datas.push_back(euler_rpy_[0]);
    datas.push_back(euler_rpy_[1]);
    datas.push_back(euler_rpy_[2]);

    // Proj gravity
    datas.push_back(proj_gravity_[0]);
    datas.push_back(proj_gravity_[1]);
    datas.push_back(proj_gravity_[2]);

    if (log_flag_ & LOG_DATA_FLAG_CSV)
    {
      csv_logger_->Write(datas);
    }

    if (log_flag_ & (LOG_DATA_FLAG_DDS))
    {
      dds_logger_->Write(datas);
    }
  }

} // namespace ovinf

#endif // !ROBOT_EFC_HPP

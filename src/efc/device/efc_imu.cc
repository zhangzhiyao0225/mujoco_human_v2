#include "efc_imu.h"

#include <Eigen/Geometry>

namespace bitbot {

EfcImu::EfcImu(pugi::xml_node const& device_node) : EfcDevice(device_node) {
  basic_type_ = (uint32_t)BasicDeviceType::IMU;
  type_ = (uint32_t)EfcDeviceType::EFC_IMU;

  monitor_header_.headers = {"roll",  "pitch",  "yaw",    "acc_x", "acc_y",
                             "acc_z", "gyro_x", "gyro_y", "gyro_z"};
  monitor_data_.resize(monitor_header_.headers.size());
  ConfigParser::ParseAttribute2ui(dds_imu_index_,
                                  device_node.attribute("dds_imu_index"));
}

EfcImu::~EfcImu() {}

void Rot2RPY(Eigen::Matrix3f const& rot_mat, Eigen::Vector3f& res) {
  constexpr int row_num = 3;
  float roll_ = std::atan2(rot_mat(2, 1), rot_mat(2, 2));
  float pitch_ = std::atan2(
      -rot_mat(2, 0),
      sqrt(rot_mat(2, 1) * rot_mat(2, 1) + rot_mat(2, 2) * rot_mat(2, 2)));
  float yaw_ = std::atan2(rot_mat(1, 0), rot_mat(0, 0));
  res << roll_, pitch_, yaw_;
}

void EfcImu::Input(const DdsInterface::Ptr dds_interface) {
  auto const& this_imu = dds_interface->GetRobotData()->imu()[dds_imu_index_];
  Eigen::Matrix3f Rbi;
  // Imu frame w.r.t. body
  // Rbi << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Rbi << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  Eigen::Matrix3f Rwi;
  Eigen::AngleAxisf roll(this_imu.Roll(), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch(this_imu.Pitch(), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw(this_imu.Yaw(), Eigen::Vector3f::UnitZ());
  Rwi = yaw * pitch * roll;
  Eigen::Matrix3f Rwb = Rwi * Rbi.transpose();
  Eigen::Vector3f rpy_true;
  Rot2RPY(Rwb, rpy_true);
  roll_ = rpy_true(0);
  pitch_ = rpy_true(1);
  yaw_ = rpy_true(2);

  Eigen::Vector3f a_i(this_imu.A_x(), this_imu.A_y(), this_imu.A_z());
  Eigen::Vector3f a_b = Rbi * a_i;
  acc_x_ = a_b(0);
  acc_y_ = a_b(1);
  acc_z_ = a_b(2);

  Eigen::Vector3f w_i(this_imu.W_x(), this_imu.W_y(), this_imu.W_z());
  Eigen::Vector3f w_b = Rbi * w_i;
  gyro_x_ = w_b(0);
  gyro_y_ = w_b(1);
  gyro_z_ = w_b(2);
}

void EfcImu::Output(const DdsInterface::Ptr) {}

void EfcImu::UpdateRuntimeData() {
  constexpr double rad2deg = 180.0 / M_PI;

  monitor_data_[0] = rad2deg * roll_;
  monitor_data_[1] = rad2deg * pitch_;
  monitor_data_[2] = rad2deg * yaw_;
  monitor_data_[3] = acc_x_;
  monitor_data_[4] = acc_y_;
  monitor_data_[5] = acc_z_;
  monitor_data_[6] = gyro_x_;
  monitor_data_[7] = gyro_y_;
  monitor_data_[8] = gyro_z_;
}

void EfcImu::UpdateModel(const DdsInterface::Ptr dds_interface) {}

}  // namespace bitbot

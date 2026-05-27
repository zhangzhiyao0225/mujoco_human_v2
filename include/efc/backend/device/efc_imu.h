#ifndef EFC_IMU_H
#define EFC_IMU_H

#include "device/efc_device.hpp"

namespace bitbot {

class EfcImu final : public EfcDevice {
 public:
  EfcImu(pugi::xml_node const& device_node);
  ~EfcImu();

  inline double GetRoll() { return roll_; }

  inline double GetPitch() { return pitch_; }

  inline double GetYaw() { return yaw_; }

  inline double GetAccX() { return acc_x_; }

  inline double GetAccY() { return acc_y_; }

  inline double GetAccZ() { return acc_z_; }

  inline double GetGyroX() { return gyro_x_; }

  inline double GetGyroY() { return gyro_y_; }

  inline double GetGyroZ() { return gyro_z_; }

 private:
  virtual void Input(const DdsInterface::Ptr dds_interface) final;
  virtual void Output(const DdsInterface::Ptr dds_interface) final;
  virtual void UpdateModel(const DdsInterface::Ptr dds_interface) final;
  virtual void UpdateRuntimeData() final;

  unsigned int dds_imu_index_ = 0;

  double roll_ = 0;
  double pitch_ = 0;
  double yaw_ = 0;
  double acc_x_ = 0;
  double acc_y_ = 0;
  double acc_z_ = 0;
  double gyro_x_ = 0;
  double gyro_y_ = 0;
  double gyro_z_ = 0;
};

}  // namespace bitbot

#endif  // !EFC_IMU_H

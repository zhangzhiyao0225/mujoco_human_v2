#ifndef EFC_DEVICE_HPP
#define EFC_DEVICE_HPP

#include "dds/dds_interface.hpp"
#include "device/device.hpp"

namespace bitbot {

enum class EfcDeviceType : uint32_t {
  EFC_DEVICE = 12000,
  EFC_JOINT,
  EFC_IMU,
};

class EfcDevice : public Device {
 public:
  EfcDevice(const pugi::xml_node& device_node) : Device(device_node) {}
  ~EfcDevice() = default;

  // Method to w/r efc
  virtual void UpdateModel(const DdsInterface::Ptr dds_interface) = 0;
  virtual void Input(const DdsInterface::Ptr dds_interface) = 0;
  virtual void Output(const DdsInterface::Ptr dds_interface) = 0;

  // Inherited from Device base calss
  virtual void UpdateRuntimeData() = 0;

 private:
};
}  // namespace bitbot

#endif  // !EFC_DEVICE_HPP

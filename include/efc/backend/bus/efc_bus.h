#ifndef EFC_BUS_H
#define EFC_BUS_H

#include "bus/bus_manager.hpp"
#include "dds/dds_interface.hpp"
#include "device/efc_device.hpp"
#include "device/efc_imu.h"
#include "device/efc_joint.h"

namespace bitbot {
class EfcBus : public BusManagerTpl<EfcBus, EfcDevice> {
 public:
  EfcBus();
  ~EfcBus();

  void WriteBus();
  void ReadBus();
  void UpdateDevices();

  inline void SetInterface(const DdsInterface::Ptr dds_interface) {
    dds_interface_ = dds_interface;
  }

 protected:
  void doConfigure(const pugi::xml_node& bus_node);
  void doRegisterDevices();

 private:
  DdsInterface::Ptr dds_interface_;
};
}  // namespace bitbot

#endif  // !EFC_BUS_H

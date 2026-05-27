#include "bus/efc_bus.h"

namespace bitbot
{

  EfcBus::EfcBus() {}

  EfcBus::~EfcBus() {}

  void EfcBus::doConfigure(const pugi::xml_node &bus_node)
  {
    CreateDevices(bus_node);
  }

  void EfcBus::doRegisterDevices()
  {
    static DeviceRegistrar<EfcDevice, EfcJoint> efc_joint(
        (uint32_t)EfcDeviceType::EFC_JOINT, "EfcJoint");
    static DeviceRegistrar<EfcDevice, EfcImu> efc_imu(
        (uint32_t)EfcDeviceType::EFC_IMU, "EfcImu");
  }

  void EfcBus::WriteBus()
  {
    // TODO: REMOVE return;
    // return;
    for (auto &device : devices_)
    {
      device->Output(dds_interface_);
    }

    dds_interface_->GetCtrlCmd()->timestamp() =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    dds_interface_->PublishJointCommand();
  }

  void EfcBus::ReadBus()
  {
    // TODO: REMOVE return;
    // return;
    dds_interface_->UpdateRobotData();
    // auto message_time = dds_interface_->GetRobotData()->timestamp();
    //
    // auto this_time = std::chrono::duration_cast<std::chrono::milliseconds>(
    //                      std::chrono::system_clock::now().time_since_epoch())
    //                      .count();
    //
    // std::cout << "[EfcBus] DDS Message Delay: " << this_time - message_time
    //           << " ms" << std::endl;

    for (auto &device : devices_)
    {
      device->Input(dds_interface_);
    }
  }

  void EfcBus::UpdateDevices()
  {
    // TODO: REMOVE return;
    // return;
    dds_interface_->UpdateRobotData();
    auto ctrl_cmd = dds_interface_->GetCtrlCmd();
    ctrl_cmd->emergency_stop() = false;
    ctrl_cmd->power_on() = false;

    for (auto &device : devices_)
    {
      device->UpdateModel(dds_interface_);
    }
  }

} // namespace bitbot

#pragma once

#include <algorithm>
#include <map>
#include <optional>
#include <pugixml.hpp>
#include <vector>

#include "types.hpp"
#include "device/device.hpp"
#include "device/device_factory.hpp"
#include "kernel/config_parser.h"

namespace bitbot
{

  struct BusMonitorHeader
  {
    std::vector<DeviceMonitorHeader> devices;

    struct glaze
    {
      using T = BusMonitorHeader;
      static constexpr auto value = glz::object("devices", &T::devices);
    };
  };

  template <typename DerivedT, typename DeviceT>
  class BusManagerTpl
  {
  public:
    BusManagerTpl() : logger_(Logger().ConsoleLogger())
    {
      bus_monitor_data_.reserve(1024);

      RegisterDevices();
    }

    ~BusManagerTpl() { DeleteAllDevices(); }

    void ReadBus() { derived().ReadBus(); }

    void WriteBus() { derived().WriteBus(); }

    void RegisterDevices() { Accessor::aRegisterDevices(derived()); }

    void Configure(const pugi::xml_node &bus_node)
    {
      Accessor::aConfigure(derived(), bus_node);
      // derived().RegisterDevices();
      // CreateDevices(bus_node);

      GenerateHeaders();
    }

    void GenerateHeaders()
    {
      devices_csv_headers_.clear();

      std::vector<std::string> csv_headers;
      for (auto &device : devices_)
      {
        for (auto &header : device->MonitorHeader().headers)
        {
          csv_headers.push_back(device->MonitorHeader().name + "_" + header);
        }
        bus_monitor_header_.devices.push_back(device->MonitorHeader());
      }
      devices_csv_headers_.insert(devices_csv_headers_.end(), csv_headers.begin(),
                                  csv_headers.end());
    }

    const BusMonitorHeader &MonitorHeader() { return bus_monitor_header_; }

    const std::vector<Number> &MonitorData() { return bus_monitor_data_; }

    const std::vector<std::string> &DevicesCSVHeaders()
    {
      return devices_csv_headers_;
    }

    void UpdateRuntimeData()
    {
      bus_monitor_data_.clear();

      // TODO: REMOVE return;
      // return;

      for (auto &device : devices_)
      {
        device->UpdateRuntimeData();
        bus_monitor_data_.insert(bus_monitor_data_.end(),
                                 device->MonitorData().begin(),
                                 device->MonitorData().end());
      }
    }

    void CreateDevices(const pugi::xml_node &bus_node)
    {
      pugi::xml_node device_node = bus_node.child("device");
      if (bus_node.child("device").type() == pugi::node_null)
      {
        logger_->error("no device");

        return;
      }

      std::string device_type_str;
      while (device_node != NULL)
      {
        ConfigParser::ParseAttribute2s(device_type_str,
                                       device_node.attribute("type"));
        DeviceT *device = DeviceFactory<DeviceT>::Instance().CreateDevice(
            device_type_str, device_node);
        logger_->info("find device: id: {} name: {}", device->Id(),
                      device->Name());
        devices_.push_back(device);

        device_node = device_node.next_sibling("device");
      }

      devices_num_ = devices_.size();

      for (auto &device : devices_)
      {
        id_device_map_[device->Id()] = device;
      }
    }

    template <typename T>
    std::optional<T *> GetDevice(unsigned int id) const
    {
      auto device_i = id_device_map_.find(id);
      if (device_i == id_device_map_.end())
      {
        logger_->error("Failed to get device. id {} is out of range.", id);
        return std::nullopt;
      }
      else
      {
        auto device = dynamic_cast<T *>(device_i->second);
        if (device == nullptr)
        {
          logger_->error(
              "Failed to get device {}. Error device type. Actual device type is "
              "{}.",
              id, device_i->second->Type());
          return std::nullopt;
        }
        else
        {
          std::optional<T *> val(device);
          return val;
        }
      }
    }

  protected:
    void DeleteAllDevices()
    {
      for (auto &device : devices_)
      {
        delete device;
      }
      devices_.clear();
    }

    std::vector<DeviceT *> devices_;
    std::map<unsigned int, DeviceT *> id_device_map_;

    size_t devices_num_ = 0;

    BusMonitorHeader bus_monitor_header_;
    std::vector<Number> bus_monitor_data_;
    std::vector<std::string> devices_csv_headers_;

    SpdLoggerSharedPtr logger_;

  private:
    DerivedT &derived() { return static_cast<DerivedT &>(*this); }

    struct Accessor : DerivedT
    {
      static void aRegisterDevices(DerivedT &derived)
      {
        void (DerivedT::*fn)() = &Accessor::doRegisterDevices;
        return (derived.*fn)();
      }

      static void aConfigure(DerivedT &derived, const pugi::xml_node &bus_node)
      {
        void (DerivedT::*fn)(const pugi::xml_node &) = &Accessor::doConfigure;
        return (derived.*fn)(bus_node);
      }
    };
  };

} // namespace bitbot

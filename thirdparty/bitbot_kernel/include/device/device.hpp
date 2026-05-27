#pragma once

#include <memory>
#include <pugixml.hpp>
#include <string>
#include <vector>

#include "types.hpp"

#include "glaze/glaze.hpp"
#include "kernel/config_parser.h"
#include "utils/logger.h"

namespace bitbot
{
  enum class BasicDeviceType : uint32_t
  {
    NONE = 0,
    MOTOR,
    IMU,
    SENSOR,
    FORCE_SENSOR,
    CONTACT_SENSOR,
    POSITION_SENSOR,
    USER_DEFINE = 10000
  };
  // enum class DeviceType : uint32_t
  // {
  //   NONE = 0,
  //   MOTOR = 1000,
  //   MOTOR_ELMO,
  //   MOTOR_ELMO_SIMPLIFY,
  //   MOTOR_COPPELIASIM,
  //   MOTOR_SPECIAL_POSEIDON_ELBOW,
  //   IMU = 2000,
  //   IMU_MTI300,
  //   IMU_OPTIC,
  //   FORCE_SENSOR = 3000,
  //   FORCE_SRI6D
  // };

  struct DeviceMonitorHeader
  {
    std::string name;
    std::string type;
    std::vector<std::string> headers;

    struct glaze
    {
      using T = DeviceMonitorHeader;
      static constexpr auto value =
          glz::object("name", &T::name, "type", &T::type, "headers", &T::headers);
    };
  };

  class Device
  {
  public:
    Device(const pugi::xml_node &device_node)
        : logger_(Logger().ConsoleLogger())
    {
      ConfigParser::ParseAttribute2ui(id_, device_node.attribute("id"));
      ConfigParser::ParseAttribute2s(type_name_, device_node.attribute("type"));
      ConfigParser::ParseAttribute2s(name_, device_node.attribute("name"));

      monitor_header_.name = name_;
      monitor_header_.type = type_name_;
    }
    virtual ~Device() = default;

    unsigned int Id() { return id_; }

    std::string Name() { return name_; }

    uint32_t BasicType() { return basic_type_; }

    uint32_t Type() { return type_; }

    std::string TypeName() { return type_name_; }

    const DeviceMonitorHeader &MonitorHeader() { return monitor_header_; }

    const std::vector<Number> &MonitorData() { return monitor_data_; }

    virtual void UpdateRuntimeData() = 0;

    // /**
    //  * @brief 总线输入设备的数据
    //  *
    //  * @param bus_data
    //  */
    // virtual void Input(void* bus_data) = 0;

    // /**
    //  * @brief 输出到总线的数据
    //  *
    //  * @return void*
    //  */
    // virtual void* Output() = 0;

  protected:
    uint32_t basic_type_ = 0;
    uint32_t type_ = 0;
    unsigned int id_ = 0;
    std::string name_;
    std::string type_name_;

    DeviceMonitorHeader monitor_header_;
    std::vector<Number> monitor_data_;

    SpdLoggerSharedPtr logger_;
  };

} // namespace bitbot

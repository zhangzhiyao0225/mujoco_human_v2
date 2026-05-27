#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <pugixml.hpp>
#include <unordered_map>
#include <vector>

#include "utils/logger.h"

namespace bitbot {

struct DeviceConfigs {
  std::vector<pugi::xml_node> motors;
  std::vector<pugi::xml_node> imus;
  std::vector<pugi::xml_node> force_sensors;
};

class ConfigParser {
 public:
  ConfigParser();

  void Parse(std::string file);

  std::filesystem::path FilePath();

  const pugi::xml_node GetBitbotNode();
  const pugi::xml_node GetBusNode();

  std::vector<pugi::xml_node> GetMotorConfigs();
  std::vector<pugi::xml_node> GetImuConfigs();
  std::vector<pugi::xml_node> GetForceSensorConfigs();

  static void ParseAttribute2b(bool& val, const pugi::xml_attribute& attr);
  static void ParseAttribute2s(
      std::string& val,
      const pugi::xml_attribute& attr);  // 将节点属性解析到string类型
  static void ParseAttribute2ui(
      unsigned int& val,
      const pugi::xml_attribute& attr);  // 将节点属性解析到uint类型
  static void ParseAttribute2i(
      int& val, const pugi::xml_attribute& attr);  // 将节点属性解析到int类型
  static void ParseAttribute2i(
      std::optional<int>& val,
      const pugi::xml_attribute& attr);  // 将节点属性解析到int类型
  static void ParseAttribute2d(
      double& val,
      const pugi::xml_attribute& attr);  // 将节点属性解析到double类型

  static void ParseNodeText2b(bool& val, const pugi::xml_node& node);
  static void ParseNodeText2s(
      std::string& val,
      const pugi::xml_node& node);  // 将节点文本解析到string类型
  static void ParseNodeText2ui(
      unsigned int& val,
      const pugi::xml_node& node);  // 将节点文本解析到uint类型
  static void ParseNodeText2i(
      int& val, const pugi::xml_node& node);  // 将节点文本解析到int类型
  static void ParseNodeText2d(
      double& val, const pugi::xml_node& node);  // 将节点文本解析到double类型

 private:
  /**
   * @brief 解析所有设备
   *
   */
  void ParseDevices();
  void ParseMotors();
  void ParseIMUs();
  void ParseForceSensors();

  std::string file_;
  pugi::xml_document doc_;

  DeviceConfigs device_configs_;

  SpdLoggerSharedPtr logger_;
};

}  // namespace bitbot

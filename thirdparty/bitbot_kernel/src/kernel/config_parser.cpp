#include "kernel/config_parser.h"

using namespace pugi;

namespace bitbot {
ConfigParser::ConfigParser() { logger_ = Logger().ConsoleLogger(); }

std::filesystem::path ConfigParser::FilePath() {
  return std::filesystem::path(file_);
}

void ConfigParser::Parse(std::string file) {
  file_ = file;
  pugi::xml_parse_result result = doc_.load_file(file_.c_str());
  if (result.status != xml_parse_status::status_ok) {
    logger_->error("load xml file failed. {}", result.description());
  }

  if (doc_.child("bitbot").type() == pugi::node_null) {
    logger_->error("This is not a bitbot config file.");
  }
}

const pugi::xml_node ConfigParser::GetBitbotNode() {
  return doc_.child("bitbot");
}

const pugi::xml_node ConfigParser::GetBusNode() {
  return doc_.child("bitbot").child("bus");
}

void ConfigParser::ParseDevices() {
  this->ParseMotors();
  this->ParseIMUs();
  this->ParseForceSensors();
}

std::vector<pugi::xml_node> ConfigParser::GetMotorConfigs() {
  return device_configs_.motors;
}
std::vector<pugi::xml_node> ConfigParser::GetImuConfigs() {
  return device_configs_.imus;
}
std::vector<pugi::xml_node> ConfigParser::GetForceSensorConfigs() {
  return device_configs_.force_sensors;
}

void ConfigParser::ParseAttribute2b(bool& val,
                                    const pugi::xml_attribute& attr) {
  if (attr != NULL) {
    val = attr.as_bool();
  }
}
void ConfigParser::ParseAttribute2s(std::string& val,
                                    const pugi::xml_attribute& attr) {
  if (attr != NULL) {
    val = attr.as_string();
  }
}
void ConfigParser::ParseAttribute2ui(unsigned int& val,
                                     const pugi::xml_attribute& attr) {
  if (attr != NULL) {
    val = attr.as_uint();
  }
}
void ConfigParser::ParseAttribute2i(int& val, const pugi::xml_attribute& attr) {
  if (attr != NULL) {
    val = attr.as_int();
  }
}
void ConfigParser::ParseAttribute2i(std::optional<int>& val,
                                    const pugi::xml_attribute& attr) {
  if (attr != NULL) {
    val = attr.as_int();
  } else {
    val = std::nullopt;
  }
}
void ConfigParser::ParseAttribute2d(double& val,
                                    const pugi::xml_attribute& attr) {
  if (attr != NULL) {
    val = attr.as_double();
  }
}

void ConfigParser::ParseNodeText2s(std::string& val,
                                   const pugi::xml_node& node) {
  if (node != NULL) {
    if (!node.text().empty()) val = node.text().get();
  }
}
void ConfigParser::ParseNodeText2b(bool& val, const pugi::xml_node& node) {
  if (node != NULL) {
    if (!node.text().empty()) {
      if (node.text().as_int() == 1)
        val = true;
      else
        val = false;
    }
  }
}
void ConfigParser::ParseNodeText2ui(unsigned int& val,
                                    const pugi::xml_node& node) {
  if (node != NULL) {
    if (!node.text().empty()) val = node.text().as_uint();
  }
}
void ConfigParser::ParseNodeText2i(int& val, const pugi::xml_node& node) {
  if (node != NULL) {
    if (!node.text().empty()) val = node.text().as_int();
  }
}
void ConfigParser::ParseNodeText2d(double& val, const pugi::xml_node& node) {
  if (node != NULL) {
    if (!node.text().empty()) val = node.text().as_double();
  }
}

void ConfigParser::ParseMotors() {
  pugi::xml_node motor_node =
      doc_.child("bitbot").child("device").child("motor");
  while (motor_node != NULL) {
    device_configs_.motors.push_back(motor_node);

    logger_->info("find motor: id {}", motor_node.attribute("id").as_int());

    motor_node = motor_node.next_sibling("motor");
  }
}

void ConfigParser::ParseIMUs() {
  pugi::xml_node imu_node = doc_.child("bitbot").child("device").child("imu");

  while (imu_node != NULL) {
    device_configs_.imus.push_back(imu_node);

    logger_->info("find imu: {}", imu_node.attribute("id").as_int());

    imu_node = imu_node.next_sibling("imu");
  }
}

void ConfigParser::ParseForceSensors() {
  pugi::xml_node force_sensor_node =
      doc_.child("bitbot").child("device").child("force_sensor");

  while (force_sensor_node != NULL) {
    device_configs_.force_sensors.push_back(force_sensor_node);

    logger_->info("find force_sensor: {}",
                  force_sensor_node.attribute("id").as_int());

    force_sensor_node = force_sensor_node.next_sibling("force_sensor");
  }
}

}  // namespace bitbot
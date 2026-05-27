#pragma once

#include <memory>
#include <pugixml.hpp>

#include "device/device.hpp"
#include "utils/logger.h"

namespace bitbot {
/**
 * @brief 设备注册模板接口类
 *
 * @tparam DeviceTypeT 设备抽象类，某一种设备的基类
 */
template <class DeviceTypeT>
class DeviceRegistrarBase {
 public:
  // 获取设备对象抽象接口
  virtual DeviceTypeT* CreateDevice(const pugi::xml_node& node) = 0;

 protected:
  // 禁止外部构造和虚构, 子类的"内部"的其他函数可以调用
  DeviceRegistrarBase() {}
  virtual ~DeviceRegistrarBase() {}

 private:
  // 禁止外部拷贝和赋值操作
  DeviceRegistrarBase(const DeviceRegistrarBase&);
  const DeviceRegistrarBase& operator=(const DeviceRegistrarBase&);
};

/**
 * @brief 单例工厂模板类，用于获取和注册设备对象
 *
 * @tparam DeviceTypeT 设备抽象类，某一种设备的基类
 */
template <class DeviceTypeT>
class DeviceFactory {
 public:
  // 获取工厂单例
  static DeviceFactory<DeviceTypeT>& Instance() {
    static DeviceFactory<DeviceTypeT> instance;
    return instance;
  }

  // 设备注册
  void RegisterDevice(DeviceRegistrarBase<DeviceTypeT>* registrar,
                      uint32_t device_type, std::string type_name) {
    device_registry_[device_type] = registrar;
    device_name_type_[type_name] = device_type;
  }

  // 根据device_type，获取对应具体的设备对象
  DeviceTypeT* CreateDevice(uint32_t device_type, const pugi::xml_node& node) {
    // 从map找到已经注册过的设备，并返回设备对象
    if (device_registry_.find(device_type) != device_registry_.end()) {
      return device_registry_[device_type]->CreateDevice(node);
    }

    // 未注册的设备，则报错未找到
    logger_->error("Unkown device type: {}", static_cast<int>(device_type));

    return NULL;
  }

  // 根据名字，获取对应具体的设备对象
  DeviceTypeT* CreateDevice(std::string type_name, const pugi::xml_node& node) {
    // 从map找到已经注册过的设备，并返回设备对象
    if (device_name_type_.find(type_name) != device_name_type_.end()) {
      return CreateDevice(device_name_type_[type_name], node);
    }

    // 未注册的设备，则报错未找到
    logger_->error("Unkown device type name: {}", type_name);

    return NULL;
  }

  size_t DeviceNum() { return device_registry_.size(); }

 private:
  // 禁止外部构造和虚构
  DeviceFactory() : logger_(Logger().ConsoleLogger()) {}
  ~DeviceFactory() {}

  // 禁止外部拷贝和赋值操作
  DeviceFactory(const DeviceFactory&);
  const DeviceFactory& operator=(const DeviceFactory&);

  // 保存注册过的设备，key:设备名字 , value:设备类型
  std::map<uint32_t, DeviceRegistrarBase<DeviceTypeT>*> device_registry_;
  std::map<std::string, uint32_t> device_name_type_;
  Logger::Console logger_;
};

/**
 * @brief
 * 设备注册模板类，用于创建具体设备和在工厂里注册设备。尽量创建为全局变量或静态变量或在堆上创建，避免指针失效。
 *
 * @tparam DeviceTypeT 设备抽象类（基类）
 * @tparam DeviceImplT 具体设备（设备种类的子类）
 */
template <class DeviceTypeT, class DeviceImplT>
class DeviceRegistrar : public DeviceRegistrarBase<DeviceTypeT> {
 public:
  // 构造函数，用于注册设备到工厂，只能显示调用
  explicit DeviceRegistrar(uint32_t device_type, std::string type_name) {
    // 通过工厂单例把设备注册到工厂
    DeviceFactory<DeviceTypeT>::Instance().RegisterDevice(this, device_type,
                                                          type_name);
  }

  // 创建具体设备对象指针
  DeviceTypeT* CreateDevice(const pugi::xml_node& node) {
    return static_cast<DeviceTypeT*>(new DeviceImplT(node));
  }
};
}  // namespace bitbot

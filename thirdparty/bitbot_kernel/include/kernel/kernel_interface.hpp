#pragma once

#include "kernel/kernel_data.hpp"

namespace bitbot {
class KernelInterface {
 public:
  KernelInterface(KernelConfigData* kernel_data, KernelRuntimeData* runtime)
      : config_data_(kernel_data), runtime_data_(runtime) {}

  /**
   * @brief 是否记录数据
   *
   * @return true: 记录数据
   * @return false: 不记录数据
   */
  bool isRecordLog() const { return config_data_->record_log_flag; }

  /**
   * @brief 获取运行周期数
   *
   * @return uint64_t 周期数
   */
  uint64_t GetPeriodsCount() const { return runtime_data_->periods_count; }

  /**
   * @brief 获取当前状态
   *
   * @return int 状态
   */
  StateId GetState() const { return runtime_data_->state; }

  /**
   * @brief 获取机器人运行周期
   *
   * @return double
   */
  double GetPeriod() const { return config_data_->period; }

  void EmitEvent(EventId id, EventValue value) const {
    runtime_data_->events_queue.enqueue(
        std::pair<EventId, EventValue>(id, value));
  }

 private:
  KernelConfigData* config_data_;
  KernelRuntimeData* runtime_data_;
};

}  // namespace bitbot
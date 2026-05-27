#pragma once

#include <stdint.h>
#include <time.h>

#include <queue>
#include <string>
#include <vector>

#include "types.hpp"
#include "concurrentqueue.h"

namespace bitbot
{
  struct KernelConfigData
  {
    std::string config_file = "config.xml";

    struct timespec system_time, last_system_time;
    double period = 0;
    double process_time = 0;

    bool power_on_flag = false;
    bool skip_power_on_flag = false;
    bool power_on_finish_flag = false;

    bool stop_flag = false;

    bool record_log_flag = false;
  };

  struct KernelRuntimeData
  {
    KernelRuntimeData() { monitor_data.resize(MonitorHeader().size()); }

    const std::vector<std::string> &MonitorHeader()
    {
      static std::vector<std::string> headers = {"state", "periods_count",
                                                 "period(ms)", "process_t(ms)",
                                                 "kernel_t(ms)"};
      return headers;
    }

    const std::vector<Number> &MonitorData() { return monitor_data; }

    void Update()
    {
      monitor_data[0] = state;
      monitor_data[1] = periods_count;
      monitor_data[2] = period;
      monitor_data[3] = process_time;
      monitor_data[4] = kernel_task_time;
    }

    StateId state = 0;
    uint64_t periods_count = 0;
    double period = 0;
    double process_time = 0;
    double kernel_task_time = 0;

    std::vector<Number> monitor_data;

    // std::queue<std::pair<EventId, EventValue>> events_queue_;
    moodycamel::ConcurrentQueue<std::pair<EventId, EventValue>> events_queue;
  };
} // namespace bitbot

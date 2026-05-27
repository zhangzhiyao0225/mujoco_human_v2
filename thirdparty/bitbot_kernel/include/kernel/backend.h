#ifndef BITBOT_KERNEL_BACKEND_H
#define BITBOT_KERNEL_BACKEND_H

#include <atomic>
#include <fstream>
#include <map>
#include <optional>
#include <thread>
#include <unordered_map>

#include "types.hpp"
#include "glaze/glaze.hpp"
#include "kernel/kernel_data.hpp"
#include "readerwriterqueue.h"
#include "utils/logger.h"

namespace pugi
{
  class xml_node;
} // namespace pugi

namespace bitbot
{
  class Backend
  {
  public:
    Backend(const pugi::xml_node &node, int listen_port = 12888);
    ~Backend();

    void SetPort(int port);
    void RegisterSettingsFile(std::string file);

    void SetMonitorHeaders(std::string json_str);
    void SetMonitorData(const std::vector<Number> &data);

    void SetStatesList(std::vector<std::pair<StateId, std::string>> states);
    void SetEventsMap(std::unordered_map<std::string, EventId> *events_map);

    inline moodycamel::ReaderWriterQueue<std::vector<std::pair<EventId, EventValue>>> &GetQueue()
    {
      return events_queue_;
    }
    inline bool GetEvent(std::vector<std::pair<EventId, EventValue>> &event)
    {
      return events_queue_.try_dequeue(event);
    }

    void Run();

    void Stop();

  private:
    void Running();

    void UpdateData();

    // struct BackendControlSetting
    // {
    //   std::string event;
    //   std::string kb_key;

    //   struct glaze
    //   {
    //     using T = BackendControlSetting;
    //     static constexpr auto value = glz::object(
    //         "event", &T::event,
    //         "kb_key", &T::kb_key);
    //   };
    // };

    // struct BackendSettings
    // {
    //   std::vector<BackendControlSetting> control;

    //   struct glaze
    //   {
    //     using T = BackendSettings;
    //     static constexpr auto value = glz::object(
    //         "control", &T::control);
    //   };
    // } settings_;

    Logger::Console logger_;

    bool run_ = false;
    int listen_port_;

    std::string monitor_headers_json_;
    std::atomic<bool> is_data_update_ = false;
    std::atomic<bool> is_reading_data_ = false;
    // MonitorData monitor_data_;
    std::string monitor_data_str_;

    // StatesType states_list_;
    std::string states_list_str_;

    std::unordered_map<std::string, EventId> *events_name_id_map_ = nullptr;

    moodycamel::ReaderWriterQueue<std::vector<std::pair<EventId, EventValue>>>
        events_queue_;

    std::unique_ptr<std::thread> running_thread_ = nullptr;
  };

} // namespace bitbot

#endif // BITBOT_KERNEL_BACKEND_H

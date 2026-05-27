#include "kernel/backend.h"

#include "utils/assert.h"
#include "utils/time_func.h"

/**
 * 1. init pos
 * 2. stand-walk
 * 3. walk-stand
 * 4. rubust
 */

/**
 * TODO:
 * 1. user_func.h: add states for events
 * 2. add states for publisher
 */
namespace bitbot
{
  Backend::Backend(const pugi::xml_node &node, int listen_port)
      : logger_(Logger().ConsoleLogger()),
        listen_port_(listen_port),
        events_queue_(16),
        running_thread_(nullptr)
  {
  }

  Backend::~Backend() { Stop(); }

  void Backend::SetPort(int port) { listen_port_ = port; }

  void Backend::RegisterSettingsFile(std::string file)
  {
    // auto buffer = glz::file_to_buffer(file);
    // auto ec =
    //     glz::read<glz::opts{.error_on_unknown_keys = false}>(settings_, buffer);
    // if (ec)
    // {
    //   logger_->error(glz::format_error(ec, buffer));
    // }
  }

  void Backend::SetMonitorHeaders(std::string str)
  {
    monitor_headers_json_ = str;
  }

  void Backend::SetStatesList(
      std::vector<std::pair<StateId, std::string>> states)
  {
    // for (auto &state : states)
    // {
    //   states_list_.states.push_back({.id = state.first, .name = state.second});
    // }
    // states_list_str_ = glz::write_json(states_list_);
  }

  void Backend::SetEventsMap(
      std::unordered_map<std::string, EventId> *events_map)
  {
    events_name_id_map_ = events_map;
  }

  void Backend::Run()
  {
    if (!run_)
    {
      bitbot_assert(events_name_id_map_,
                    "Backend events_name_id_map_ must have valid value");
      if (running_thread_ == nullptr)
      {
        // running_thread_ = std::make_unique<std::thread>(&Backend::Running, this);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      run_ = true;
    }
  }

  void Backend::Stop()
  {
    if (run_)
    {

      if (running_thread_ && running_thread_->joinable())
      {
        running_thread_->join();
        running_thread_ = nullptr;
      }
      run_ = false;
    }
  }

  void Backend::SetMonitorData(const std::vector<Number> &data)
  {
    if (!is_reading_data_.load())
    {
      is_reading_data_.store(true);
      // monitor_data_.data = data;
      is_reading_data_.store(false);
      is_data_update_.store(true);
    }
  }

  void Backend::Running()
  {
  }

  void Backend::UpdateData()
  {
    if (is_data_update_.load())
    {
      is_reading_data_.store(true);
      // monitor_data_str_ = glz::write_json(monitor_data_);
      is_reading_data_.store(false);
      is_data_update_.store(false);
    }
  }
} // namespace bitbot

#pragma once

#include <sys/stat.h>
#include <time.h>

#include <algorithm>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "bus/bus_manager.hpp"
#include "device/device_factory.hpp"
#include "glaze/glaze.hpp"
#include "kernel/backend.h"
#include "kernel/config_parser.h"
#include "kernel/kernel_data.hpp"
#include "kernel/kernel_file_logger.hpp"
#include "kernel/kernel_interface.hpp"
#include "utils/assert.h"
#include "utils/cpu_affinity.h"
#include "utils/extra_data.hpp"
#include "utils/logger.h"
#include "utils/time_func.h"

#include "dds/handop_ui.hpp"

class Twist;

namespace bitbot
{
  enum class KernelEvent : EventId
  {
    STOP,
    START,
    ENABLE_RECORD,
  };

  enum class KernelState : StateId
  {
    IDLE,
    STOPPED
  };

  /**
   * @brief Kernel模板类
   *
   * @tparam DerivedT
   */
  template <typename DerivedT, typename BusManagerT, typename UserData,
            CTString... CTSArray>
  class KernelTpl
  {
    DerivedT &derived() { return static_cast<DerivedT &>(*this); }

  public:
    using ExtraData = ExtraDataImpl<CTSArray...>;
    using ConfigFunc = std::function<void(const BusManagerT &, UserData &)>;
    using StartFunc = std::function<void(ExtraData &, UserData &)>;
    using FinishFunc = std::function<void(UserData &)>;
    using EventFunc =
        std::function<std::optional<StateId>(bitbot::EventValue, UserData &)>;
    using StateFunc = std::function<void(const bitbot::KernelInterface &,
                                         ExtraData &, UserData &)>;

    KernelTpl(std::string config_file)
        : logger_(Logger().ConsoleLogger()),
          config_file_(config_file),
          kernel_interface_(&kernel_config_data_, &kernel_runtime_data_),
          parser_(std::make_unique<ConfigParser>())
    {
      using namespace std::filesystem;

      path config_file_path(config_file_);
      try
      {
        if (config_file_path.is_relative())
        {
          config_file_path = canonical(config_file_path);
        }
        config_file_ = config_file_path.string();
      }
      catch (const std::exception &ex)
      {
        logger_->error(
            "Bitbot config file: \"{}\" can\'t convert to an absolute path. {}",
            config_file, ex.what());
        std::abort();
      }

      KernelConfigure(config_file_);

      if (backend_port_)
      {
        backend_->SetPort(backend_port_);
      }
      backend_->RegisterSettingsFile(backend_settings_file_);

      KernelRegisterEvent(
          "stop", static_cast<EventId>(KernelEvent::STOP),
          [this](EventValue, UserData &) -> std::optional<StateId>
          {
            this->kernel_config_data_.stop_flag = true;
            return static_cast<StateId>(KernelState::STOPPED);
          },
          true);
      KernelRegisterEvent(
          "enable_record", static_cast<EventId>(KernelEvent::ENABLE_RECORD),
          [this](EventValue, UserData &) -> std::optional<StateId>
          {
            this->kernel_config_data_.record_log_flag = true;
            this->runtime_data_logger_->Enable();
            return std::nullopt;
          },
          true);
      KernelRegisterEvent(
          "start", static_cast<EventId>(KernelEvent::START),
          [this](EventValue evalue, UserData &) -> std::optional<StateId>
          {
            if (1) // (evalue == static_cast<EventValue>(KeyboardEvent::Up))
            {
              accessor::AdoStart(this->derived());
              if (this->start_func_)
              {
                start_func_(this->extra_data_, this->user_data_);
              }
              return this->user_first_state_.value();
            }
            else
              return std::nullopt;
          });

      KernelRegisterState("kernel_idle", static_cast<StateId>(KernelState::IDLE),
                          [this](const bitbot::KernelInterface &kernel,
                                 ExtraData &extra_data, UserData &user_data) {},
                          {static_cast<EventId>(KernelEvent::START),
                           static_cast<EventId>(KernelEvent::STOP),
                           static_cast<EventId>(KernelEvent::ENABLE_RECORD)});
      KernelRegisterState("kernel_stopped",
                          static_cast<StateId>(KernelState::STOPPED),
                          [this](const bitbot::KernelInterface &kernel,
                                 ExtraData &extra_data, UserData &user_data)
                          {
                            logger_->info("kernel stopped");
                            std::cout << "kernel stopped" << std::endl;
                          },
                          {});
    }

    ~KernelTpl() {}

    void RegisterConfigFunc(ConfigFunc func) { config_func_ = func; }

    void RegisterStartFunc(StartFunc func) { start_func_ = func; }

    void RegisterFinishFunc(FinishFunc func) { finish_func_ = func; }

    void RegisterEvent(std::string name, EventId id, EventFunc func,
                       bool always_enabled = false)
    {
      bitbot_assert(events_name_id_map_.find(name) == events_name_id_map_.end(),
                    std::format("event name: {} has exist", name));
      bitbot_assert(events_.find(id) == events_.end(),
                    std::format("event id: {} has exist", id));
      if (id < 1000)
      {
        logger_->warn("user defined event: {} id: {} less than 1000", name, id);
      }

      doRegisterEvent(name, id, func, always_enabled);
    }

    void SetFirstState(StateId id)
    {
      user_first_state_ = id;
      std::cout << "first state: " << id << std::endl;
      current_state_ = &states_.at(id);
    }

    void RegisterState(std::string name, StateId id, StateFunc func,
                       std::vector<EventId> events)
    {
      bitbot_assert(states_name_id_map_.find(name) == states_name_id_map_.end(),
                    std::format("state name: {} has exist", name));
      bitbot_assert(states_.find(id) == states_.end(),
                    std::format("state id: {} has exist", id));
      if (id < 1000)
      {
        logger_->warn("user defined state: {}, id: {} should larger than 1000",
                      name, id);
      }

      doRegisterState(name, id, func, events);
    }

    void Run()
    {
      GenerateHeaders();
      RegenerateStatesAccessibalEvents();
      this->runtime_data_logger_ = std::make_unique<KernelFileLogger>(
          runtime_data_logger_filename_, csv_headers_);
      backend_->SetMonitorHeaders(monitor_headers_json_str_);
      std::vector<std::pair<StateId, std::string>> states_list;
      for (auto &state : states_)
      {
        states_list.push_back(
            std::pair<StateId, std::string>(state.first, state.second.name));
      }
      backend_->SetStatesList(states_list);
      backend_->SetEventsMap(&events_name_id_map_);
      backend_->Run();

      if (config_func_)
        config_func_(busmanager_, user_data_);

      bitbot_assert(
          user_first_state_.has_value(),
          "User's first state is empty. Need invoke SetFirstState(StateId id).");
      bitbot_assert(states_.find(user_first_state_.value()) != states_.end(),
                    "The id of first state is invalid.");

      // std::cout << "first state: 0" << std::endl;

      current_state_ = &states_.at(static_cast<StateId>(KernelState::IDLE));

      accessor::AdoRun(this->derived());

      if (finish_func_)
        finish_func_(user_data_);
    }

    void EmitEvent(EventId id, EventValue value)
    {
      this->kernel_runtime_data_.events_queue.enqueue(
          std::pair<EventId, EventValue>(id, value));
    }

    handop::HandOpHandler::Ptr GetRobNotiPublisher()
    {
      return handop_;
    }

  protected:
    void KernelLoopTask()
    {
      busmanager_.ReadBus();

      current_state_->func(kernel_interface_, extra_data_, user_data_);

      busmanager_.WriteBus();
    }

    /**
     * @brief kernel配置文件
     *
     * @param file_name 配置文件名称
     */
    void KernelConfigure(std::string file_name)
    {
      using namespace std::filesystem;

      // 解析文件节点
      parser_->Parse(file_name);
      this->backend_ = std::make_unique<Backend>(parser_->GetBitbotNode());

      // backend
      if (auto backend_node = parser_->GetBitbotNode().child("backend");
          backend_node != NULL)
      {
        ConfigParser::ParseAttribute2i(backend_port_,
                                       backend_node.attribute("port"));
        ConfigParser::ParseAttribute2s(backend_settings_file_,
                                       backend_node.attribute("settings_file"));
        if (backend_settings_file_.empty())
        {
          logger_->error("backend need attribute \'settings_file\'.");
        }
        else
        {
          path settings_file_path(backend_settings_file_);
          if (settings_file_path.is_relative())
          {
            settings_file_path =
                canonical(parser_->FilePath().parent_path() / settings_file_path);
            backend_settings_file_ = settings_file_path.string();
          }
        }
        logger_->debug("backend setting file: {}", backend_settings_file_);
      }

      // logger
      if (auto logger_node = parser_->GetBitbotNode().child("logger");
          logger_node != NULL)
      {
        ConfigParser::ParseAttribute2s(logger_path_,
                                       logger_node.attribute("path"));
        std::string level;
        ConfigParser::ParseAttribute2s(level, logger_node.attribute("level"));
        if (!level.empty())
        {
          if (level.compare("off") == 0)
          {
            Logger().SetConsoleLoggerLevel(spdlog::level::off);
          }
          else if (level.compare("critical") == 0)
          {
            Logger().SetConsoleLoggerLevel(spdlog::level::critical);
          }
          else if (level.compare("error") == 0)
          {
            Logger().SetConsoleLoggerLevel(spdlog::level::err);
          }
          else if (level.compare("warn") == 0)
          {
            Logger().SetConsoleLoggerLevel(spdlog::level::warn);
          }
          else if (level.compare("info") == 0)
          {
            Logger().SetConsoleLoggerLevel(spdlog::level::info);
          }
          else if (level.compare("debug") == 0)
          {
            Logger().SetConsoleLoggerLevel(spdlog::level::debug);
          }
          else if (level.compare("trace") == 0)
          {
            Logger().SetConsoleLoggerLevel(spdlog::level::trace);
          }
        }
      }
      if (logger_path_.empty())
      {
#ifdef _MSC_VER
        char home_path[256] = {0};
        size_t ret_size = 0;
        getenv_s(&ret_size, home_path, sizeof(home_path), "HOME");
#else
        char *home_path = getenv("HOME");
#endif // _MSC_VER
        logger_path_ = std::string(home_path) + "/bitbot_log/";
      }
      path data_log_path(logger_path_ + "/data");
      if (!exists(data_log_path))
      {
        logger_->info("Directory({}) does not exist. Now creating",
                      data_log_path.string());
        create_directories(data_log_path);
      }
      path kernel_file_log_path(logger_path_ + "/kernel");
      if (!exists(kernel_file_log_path))
      {
        logger_->info("Directory({}) does not exist. Now creating",
                      kernel_file_log_path.string());
        create_directories(kernel_file_log_path);
      }

      time_t now = time(0);
#ifdef _MSC_VER
      tm tm_buf;
      localtime_s(&tm_buf, &now);
      std::string filetime =
          std::format("{:04d}{:02d}{:02d}_{:02d}{:02d}{:02d}",
                      tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                      tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);
#else
      tm *tm_buf = localtime(&now);
      std::string filetime =
          std::format("{:04d}{:02d}{:02d}_{:02d}{:02d}{:02d}",
                      tm_buf->tm_year + 1900, tm_buf->tm_mon + 1, tm_buf->tm_mday,
                      tm_buf->tm_hour, tm_buf->tm_min, tm_buf->tm_sec);
#endif
      runtime_data_logger_filename_ =
          path(data_log_path / std::string(filetime + ".csv")).string();
      file_logger_ = Logger().CreateFileLogger(
          path(kernel_file_log_path / std::string(filetime + ".txt")).string());
      // bus
      busmanager_.Configure(this->parser_->GetBusNode());
    }

    /**
     * @brief kernel内部任务，记录数据，更新后端数据。
     *
     */
    void KernelPrivateLoopEndTask()
    {
      auto start = std::chrono::high_resolution_clock::now();
      UpdateRuntimeData();
      backend_->SetMonitorData(monitor_data_);

      // write to local file
      if (kernel_config_data_.record_log_flag)
      {
        runtime_data_logger_->Write(monitor_data_);
      }

      auto end = std::chrono::high_resolution_clock::now();
      kernel_runtime_data_.kernel_task_time =
          std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
              .count() /
          1e6;
    }

    void UpdateRuntimeData()
    {
      kernel_runtime_data_.Update();
      busmanager_.UpdateRuntimeData();

      monitor_data_.clear();
      monitor_data_.insert(monitor_data_.end(),
                           kernel_runtime_data_.MonitorData().begin(),
                           kernel_runtime_data_.MonitorData().end());
      monitor_data_.insert(monitor_data_.end(), busmanager_.MonitorData().begin(),
                           busmanager_.MonitorData().end());
      monitor_data_.insert(monitor_data_.end(), extra_data_.Data().begin(),
                           extra_data_.Data().end());
    }

    void GenerateHeaders()
    {
      // csv headers
      csv_headers_.clear();

      csv_headers_.insert(csv_headers_.end(),
                          kernel_runtime_data_.MonitorHeader().begin(),
                          kernel_runtime_data_.MonitorHeader().end());
      csv_headers_.insert(csv_headers_.end(),
                          busmanager_.DevicesCSVHeaders().begin(),
                          busmanager_.DevicesCSVHeaders().end());
      // csv_headers_.insert(csv_headers_.end(),
      //                     extra_data_.Headers().begin(),
      //                     extra_data_.Headers().end());
      for (auto str : extra_data_.Headers())
      {
        csv_headers_.push_back(std::string(str));
      }

      // monitor headers
      MonitorHeader<sizeof...(CTSArray)> header;
      header.kernel = kernel_runtime_data_.MonitorHeader();
      header.bus = busmanager_.MonitorHeader();
      header.extra = extra_data_.Headers();

      monitor_headers_json_str_ = glz::write_json(header);
    }

    inline void HandleEvent(const std::pair<EventId, EventValue> &event)
    {
      // std::cout << "proc event on state: " << current_state_->id << "(" << current_state_->name << "), event: " << event.first << " " << event.second << std::endl;
      logger_->trace("event: {}, value: {}", events_.at(event.first).name,
                     event.second);
      file_logger_->info("event: {}, value: {}", events_.at(event.first).name,
                         event.second);

      if (auto e = current_state_->accessibal_events.find(event.first);
          e != current_state_->accessibal_events.end())
      {
        if (auto res = e->second->func(event.second, user_data_);
            res.has_value())
        {
          auto value = res.value();
          this->kernel_runtime_data_.state = value;
          if (auto state = states_.find(value); state != states_.end())
          {
            current_state_ = &(state->second);
          }
          else
          {
            logger_->warn("unknown state: {}", state->second.name);
            file_logger_->warn("unknown state: {}", state->second.name);
          }
        }
      }
    }

    void HandleEvents()
    {
      static std::vector<std::pair<EventId, EventValue>> events;
      static std::pair<EventId, EventValue> event;
      while (backend_->GetEvent(events))
      {
        for (auto &e : events)
        {
          // std::cout << "will handle event: " << e.first << " " << e.second << std::endl;
          this->HandleEvent(e);
        }
      }

      while (kernel_runtime_data_.events_queue.try_dequeue(event))
      {
        this->HandleEvent(event);
      }
    }

    inline void doRegisterEvent(const std::string &name, EventId id,
                                EventFunc func, bool always_enabled = false)
    {
      events_.insert(
          std::pair<uint32_t, Event>(id, {.id = id, .name = name, .func = func}));
      events_name_id_map_.insert(std::pair<std::string, EventId>(name, id));
      if (always_enabled)
      {
        always_enabled_events_.emplace_back(id);
      }
    }

    void KernelRegisterEvent(std::string name, EventId id, EventFunc func,
                             bool always_enabled = false)
    {
      bitbot_assert(events_name_id_map_.find(name) == events_name_id_map_.end(),
                    std::format("event name: {} has exist", name));
      bitbot_assert(events_.find(id) == events_.end(),
                    std::format("event id: {} has exist", id));
      if (id > 1000)
      {
        logger_->warn("kernel defined event: {} id: {} should less than 1000",
                      name, id);
      }

      doRegisterEvent(name, id, func, always_enabled);
    }

    void doRegisterState(std::string name, StateId id, StateFunc func,
                         std::vector<EventId> events)
    {
      std::unordered_map<EventId, Event *> accessibal_events;
      for (auto event : events)
      {
        auto event_it = events_.find(event);
        bitbot_assert(event_it != events_.end(),
                      std::format("unkown event id: {}", event));
        accessibal_events.insert(
            std::pair<EventId, Event *>(event_it->first, &(event_it->second)));
      }

      states_.insert(std::pair<StateId, State>(
          id, {.id = id,
               .name = name,
               .func = func,
               .accessibal_events = accessibal_events}));
      states_name_id_map_.insert(std::pair<std::string, StateId>(name, id));
    }

    void KernelRegisterState(std::string name, StateId id, StateFunc func,
                             std::vector<EventId> events)
    {
      bitbot_assert(states_name_id_map_.find(name) == states_name_id_map_.end(),
                    std::format("state name: {} has exist", name));
      bitbot_assert(states_.find(id) == states_.end(),
                    std::format("state id: {} has exist", id));
      if (id > 1000)
      {
        logger_->warn("kernel defined state: {}, id: {} should less than 1000",
                      name, id);
      }

      doRegisterState(name, id, func, events);
    }

    void InjectEventsToState(StateId state_id, std::vector<EventId> events_id)
    {
      auto state = states_.find(state_id);
      if (state != states_.end())
      {
        for (auto event_id : events_id)
        {
          auto event_it = events_.find(event_id);
          bitbot_assert(event_it != events_.end(),
                        std::format("unkown event id: {}", event_id));
          state->second.accessibal_events.insert(
              std::pair<uint32_t, Event *>(event_it->first, &(event_it->second)));
        }
      }
    }

    void RegenerateStatesAccessibalEvents()
    {
      for (auto &state : states_)
      {
        if (state.first != static_cast<StateId>(KernelState::IDLE))
        {
          InjectEventsToState(state.first, always_enabled_events_);
        }
      }
    }

    template <typename Enum>
    static auto Enum2Value(Enum e) -> typename std::underlying_type<Enum>::type
    {
      return static_cast<typename std::underlying_type<Enum>::type>(e);
    }

    template <typename Enum>
    static auto Value2Enum(uint8_t value) ->
        typename std::enable_if<std::is_enum<Enum>::value, Enum>::type
    {
      return static_cast<Enum>(value);
    }

    struct Event
    {
      EventId id;
      EventValue value = 0;
      std::string name;
      EventFunc func;
    };

    struct State
    {
      StateId id;
      std::string name;
      StateFunc func;
      std::unordered_map<EventId, Event *> accessibal_events;
    };

    template <size_t ExtraNum>
    struct MonitorHeader
    {
      std::vector<std::string> kernel;
      BusMonitorHeader bus;
      std::array<std::string_view, ExtraNum> extra;

      struct glaze
      {
        using T = MonitorHeader<ExtraNum>;
        static constexpr auto value =
            glz::object("kernel", &T::kernel, "bus", &T::bus, "extra", &T::extra);
      };
    };

    Logger::Console logger_;
    Logger::File file_logger_;

    std::string config_file_;

    /**
     * @brief kernel数据
     */
    KernelConfigData kernel_config_data_;
    KernelRuntimeData kernel_runtime_data_;

    UserData user_data_;

    std::unordered_map<StateId, State> states_;
    std::unordered_map<EventId, Event> events_;
    std::unordered_map<std::string, StateId> states_name_id_map_;
    std::unordered_map<std::string, EventId> events_name_id_map_;
    State *current_state_ = nullptr;
    std::optional<StateId> user_first_state_;
    std::vector<EventId> always_enabled_events_;

    ExtraData extra_data_;

    std::string monitor_headers_json_str_;
    std::vector<std::string> csv_headers_;
    std::vector<Number> monitor_data_;

    std::string logger_path_;
    int backend_port_ = 0;
    std::string backend_settings_file_;
    std::string runtime_data_logger_filename_;
    std::unique_ptr<KernelFileLogger> runtime_data_logger_;

    ConfigFunc config_func_ = nullptr;
    StartFunc start_func_ = nullptr;
    FinishFunc finish_func_ = nullptr;

    KernelInterface kernel_interface_;

    std::unique_ptr<ConfigParser> parser_ = nullptr;

    /**
     * @brief 总线设备管理
     */
    BusManagerT busmanager_;

    std::unique_ptr<Backend> backend_ = nullptr;

    handop::HandOpHandler::Ptr handop_{};

  private:
    struct accessor : DerivedT
    {
      static void AdoRun(DerivedT &derived)
      {
        void (DerivedT::*fn_dorun)() = &accessor::doRun;
        return (derived.*fn_dorun)();
      }

      static void AdoStart(DerivedT &derived)
      {
        void (DerivedT::*fn_dostart)() = &accessor::doStart;
        return (derived.*fn_dostart)();
      }
    };
  };
} // namespace bitbot

#ifndef EFC_KERNEL_HPP
#define EFC_KERNEL_HPP

#include <thread>

#include "bus/efc_bus.h"
#include "dds/dds_interface.hpp"

#include "kernel/kernel.hpp"

enum class CifxKernelEvent : uint32_t
{
  POWER_ON = 100,
  POWER_ON_FINISH,
};

enum class CifxKernelState : uint32_t
{
  POWER_ON = 100,
  POWER_ON_FINISH,
};

namespace bitbot
{
  template <typename UserData, CTString... cts>
  class EfcKernel
      : public KernelTpl<EfcKernel<UserData, cts...>, EfcBus, UserData, cts...>
  {
  public:
    EfcKernel(std::string config_file, bitbot_init_param &initparam)
        : KernelTpl<EfcKernel<UserData, cts...>, EfcBus, UserData, cts...>(
              config_file)
    {
      pugi::xml_node const &bitbot_node = this->parser_->GetBitbotNode();
      can_do_next can_do = [this, &initparam](bitbot::EventId event) -> bool
      {
        if (!this->current_state_)
        {
          std::cout << "No state" << std::endl;
          return false;
        }
        auto it = initparam.states_2_evts.find(this->current_state_->id);
        if (it == initparam.states_2_evts.end())
        {
          // std::cout << "No events for state " << this->current_state_->id << std::endl;
          return false;
        }
        // auto jt = it->second.find(event);
        // if (jt == it->second.end())
        // {
        //   std::cout << "No event permit for state" << std::endl;
        //   return false;
        // }
        return true;
      };

      initparam.can_do = std::move(can_do);

      // std::cout << "Bitbot kernel config file: " << config_file << "; create handle with backend " << this->backend_ << std::endl;
      this->handop_ = std::make_shared<handop::HandOpHandler>(this->backend_->GetQueue(), initparam, bitbot_node);

      dds_interface_ = std::make_shared<DdsInterface>(bitbot_node);

      this->KernelRegisterEvent(
          "power_on", static_cast<EventId>(CifxKernelEvent::POWER_ON),
          [this](EventValue, UserData &) -> std::optional<StateId>
          {
            this->logger_->info("joints power on");
            return static_cast<StateId>(CifxKernelState::POWER_ON);
          },
          false);

      this->KernelRegisterEvent(
          "power_on_finish",
          static_cast<EventId>(CifxKernelEvent::POWER_ON_FINISH),
          [this](EventValue, UserData &) -> std::optional<StateId>
          {
            this->logger_->info("joints power on finish");
            return static_cast<StateId>(CifxKernelState::POWER_ON_FINISH);
          },
          false);

      this->InjectEventsToState(
          static_cast<StateId>(KernelState::IDLE),
          {static_cast<EventId>(CifxKernelEvent::POWER_ON)});

      this->KernelRegisterState(
          "power on", static_cast<StateId>(CifxKernelState::POWER_ON),
          [this](const bitbot::KernelInterface &kernel, auto &extra_data,
                 UserData &user_data)
          {
            if (dds_interface_->MotorReset())
            {
              Logger().ConsoleLogger()->info("Motors reset command sent.");
              this->EmitEvent(
                  static_cast<EventId>(CifxKernelEvent::POWER_ON_FINISH), 0);
            }
          },
          {static_cast<EventId>(
              static_cast<EventId>(CifxKernelEvent::POWER_ON_FINISH))});

      this->KernelRegisterState(
          "power on finish",
          static_cast<StateId>(CifxKernelState::POWER_ON_FINISH),
          [this](const bitbot::KernelInterface &kernel, auto &extra_data,
                 UserData &user_data) {},
          {static_cast<EventId>(KernelEvent::START)});

      this->busmanager_.SetInterface(dds_interface_);
    }

    ~EfcKernel()
    {
    }

  public:
    inline void HandleRobotEvent(const std::pair<EventId, EventValue> &event)
    {
      // std::cout << "handle event: " << event.first << ", " << event.second << std::endl;

      this->HandleEvent(event);
    }

  public:
    void doStart()
    {
      std::cout << "efc kernel start..." << std::endl;
      if (dds_interface_->MotorReset())
      {
        Logger().ConsoleLogger()->info(
            "Motors set to reset mode. The robot is functional.");
      }

      if (dds_interface_->MotorCostomMode())
      {
        Logger().ConsoleLogger()->info(
            "Motors set to custom mode. The robot is functional.");
      }
      else
      {
        Logger().ConsoleLogger()->info("Motor not enabled. Testing mode.");
      }

      Logger().ConsoleLogger()->info("Kernel started.");
    }

    void doRun()
    {
      doStart();
      if (this->user_first_state_.has_value())
      {
        // std::cout << "switching " << this->current_state_->id << " to " << this->user_first_state_.value() << std::endl;
        this->SetFirstState(this->user_first_state_.value());
      }

      // Logger().ConsoleLogger()->info("Once.");
      using KernelClock = std::chrono::steady_clock;

      KernelClock::time_point this_time = KernelClock::now();
      KernelClock::time_point last_time = this_time;
      KernelClock::time_point end_time = this_time;

      Logger().ConsoleLogger()->info("Waiting for the first dds interface message...");
      // TODO: REMOVE comment
      dds_interface_->WaitingForSystemReady();
      this->busmanager_.UpdateDevices();
      Logger().ConsoleLogger()->info("System initialized.");

      while (!this->kernel_config_data_.stop_flag)
      {
        this_time = KernelClock::now();

        this->kernel_runtime_data_.periods_count++;
        this->kernel_runtime_data_.period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(this_time -
                                                                 last_time)
                .count() /
            1e6;
        last_time = this_time;

        // RobotNotification noti;
        // noti.data(R"({"data":[1,2,3,4,5,6,7,8,9,100,11,12,13,14,15,16,17,18,19,120,21,22,23,24,25,26,27,28,29,130,31,32,33,34,35,36,37,38,39,140,41,42,43,44,45,46,47,48,49,150,51,52,53,54,55,56,57,58,59,160,61,62,63,64,65,66,67,68,69,170,71,72,73,74,75,76,77,78,79,180,81,82,83,84,85,86,87,88,89,190,91,92,93,94,95,96,97,98,99,700,101,102,103,104,105,106,107,108,109,710,111,112,113,114,115,116,117,118,119,720,121,122,123,124,125,126,127,128,129,730,131,132,133,134,135,136,137,138,139,740,141,142,143,144,145,146,147,148,149,750,151,152,153,154,155,156,157,158,159,760,161,162,163,164,165,166,167,168,169,770,171,172,173,174,175,176,177,178,179,780,181,182,183,184,185,186,187,188,189,790,191,192,193,194,195,196,197,198,199,800,801,802,803,804]})");
        // this->handop_->GetRobotNotificationPublisher()->Publish(std::move(noti));

        // Logger().ConsoleLogger()->info("will HandleEvents.");
        auto handle_events_start = KernelClock::now();
        this->HandleEvents();
        auto handle_events_end = KernelClock::now();
        // Logger().ConsoleLogger()->info("will KernelLoopTask.");
        auto loop_task_start = handle_events_end;
        this->KernelLoopTask();
        auto loop_task_end = KernelClock::now();
        // Logger().ConsoleLogger()->info("will KernelPrivateLoopEndTask.");
        auto private_task_start = loop_task_end;
        this->KernelPrivateLoopEndTask();
        auto private_task_end = KernelClock::now();
        // Logger().ConsoleLogger()->info("next loop.");

        end_time = private_task_end;
        this->kernel_runtime_data_.process_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_time -
                                                                 this_time)
                .count() /
            1e6;

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                           KernelClock::now() - this_time)
                           .count();
        if (elapsed < 2000) [[likely]]
        {
          std::this_thread::sleep_for(std::chrono::microseconds(2000 - elapsed));
        }
        else
        {
          auto handle_events_elapsed =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  handle_events_end - handle_events_start)
                  .count();
          auto loop_task_elapsed =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  loop_task_end - loop_task_start)
                  .count();
          auto private_task_elapsed =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  private_task_end - private_task_start)
                  .count();
          this->logger_->warn(
              "Kernel loop over time: {} us "
              "(HandleEvents: {} us, KernelLoopTask: {} us, "
              "KernelPrivateLoopEndTask: {} us, period: {:.3f} ms)",
              elapsed, handle_events_elapsed, loop_task_elapsed,
              private_task_elapsed, this->kernel_runtime_data_.period);
        }
      }

      dds_interface_->ShutDown();
    }

  private:
    DdsInterface::Ptr dds_interface_;
  };
} // namespace bitbot

#endif // !EFC_KERNEL_HPP

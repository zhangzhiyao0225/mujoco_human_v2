#pragma once

#include "bitbot_kernel/utils/logger.h"

#ifdef __linux__
#include <pthread.h>
#include <functional>
#include <chrono>

namespace bitbot
{
  class PeriodicTaskRt
  {
  public:
    PeriodicTaskRt(std::chrono::nanoseconds period, int priority, int policy = SCHED_FIFO, bool inheritance = false)
      : logger_(Logger().ConsoleLogger())
      , period_(period)
      , running_(false)
    {
      int ec = 0;

      ec = pthread_attr_init(&attr_);
      if (ec)
      {
        logger_->error("pthread_attr_init failed\n");
      }

      ec = pthread_attr_setschedpolicy(&attr_, policy);
      if (ec)
      {
        logger_->error("pthread_attr_setschedpolicy failed\n");
      }
      
      param_.sched_priority = priority;
      ec = pthread_attr_setschedparam(&attr_, &param_);
      if (ec) {
        logger_->error("pthread_attr_setschedparam failed\n");
      }

      if(inheritance)
        ec = pthread_attr_setinheritsched(&attr_, PTHREAD_INHERIT_SCHED);
      else
        ec = pthread_attr_setinheritsched(&attr_, PTHREAD_EXPLICIT_SCHED);
      if (ec) {
        logger_->error("pthread_attr_setinheritsched failed\n");
      }
    }

    ~PeriodicTaskRt()
    {
      Stop();
    }

    void Run(std::function<void()> task_func)
    {
      this->task_func_ = task_func;
      running_.store(true);
      pthread_create(&thread_, &attr_, &PeriodicTaskRt::thread_func, this);
    }

    void Stop()
    {
      if(running_.load())
      {
        running_.store(false);
        pthread_join(thread_, NULL);
      }
    }

    void SetPeriod(std::chrono::nanoseconds period)
    {
      period_ = period;
    }

  private:
    struct time_period_info {
      struct timespec next_period;
      long period_ns;
    };

    static void* thread_func(void* arg)
    {
      PeriodicTaskRt* task = static_cast<PeriodicTaskRt*>(arg);
      task->execute();
      return NULL;
    }

    void execute() {
      struct time_period_info tinfo;
      initialize_periodic_task(&tinfo);

      while(running_.load())
      {
        task_func_();
        wait_for_period_complete(&tinfo);
      }
    }

    void initialize_periodic_task(struct time_period_info *tinfo)
    {
      tinfo->period_ns = period_.count();
      clock_gettime(CLOCK_MONOTONIC, &(tinfo->next_period));
    }

    void inc_period(struct time_period_info *tinfo)
    {
      tinfo->next_period.tv_nsec += tinfo->period_ns;
      while(tinfo->next_period.tv_nsec >= 1000000000)
      {
        tinfo->next_period.tv_sec++;
        tinfo->next_period.tv_nsec -=1000000000;
      }
    }

    void wait_for_period_complete(struct time_period_info *tinfo)
    {
      inc_period(tinfo);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tinfo->next_period, NULL);
    }

    Logger::Console logger_;
    std::chrono::nanoseconds period_;
    pthread_t thread_;
    pthread_attr_t attr_;
    struct sched_param param_;
    std::atomic_bool running_;
    std::function<void()> task_func_;
  };

}

#endif // __linux__

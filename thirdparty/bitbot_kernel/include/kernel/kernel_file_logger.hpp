#pragma once

#include <readerwriterqueue.h>

#include <algorithm>
#include <atomic>
#include <format>
#include <fstream>
#include <thread>

#include "kernel/kernel_data.hpp"
#include "types.hpp"
#include "utils/cpu_affinity.h"
#include "utils/logger.h"
#include "utils/priority.h"

namespace bitbot {
class KernelFileLogger {
 public:
  KernelFileLogger(std::string filename, std::vector<std::string> headers)
      : data_file_name_(filename),
        headers_(headers),
        headers_size_(headers_.size()),
        data_queue_(1024),
        logger_(Logger().ConsoleLogger()) {
    write_thread_ = std::thread(&KernelFileLogger::WriteData, this);
  }

  ~KernelFileLogger() {
    run_ = false;
    write_thread_.join();
    if (data_file_.is_open()) data_file_.close();
  }

  void Enable() { enable_.store(true); }

  void Write(const std::vector<Number> data) {
    if (file_bad_) return;

    data_queue_.enqueue(data);
  }

 private:
  void WriteTitle(std::vector<std::string> headers) {
    std::string title;

    std::for_each(headers.begin(), headers.end() - 1,
                  [&title](std::string& header) {
                    std::format_to(std::back_inserter(title), "{},", header);
                  });
    std::format_to(std::back_inserter(title), "{}\n", headers.back());

    data_file_.write(title.data(), title.size());
  }

  std::string DataToCSV(std::vector<Number> data) {
    static NumberVisitor visitor;
    std::string csv_str;

    std::for_each(data.begin(), data.end() - 1, [&csv_str](Number& number) {
      std::format_to(std::back_inserter(csv_str), "{},",
                     std::visit(visitor, number));
    });
    std::format_to(std::back_inserter(csv_str), "{}\n",
                   std::visit(visitor, data.back()));

    return csv_str;
  }

  void WriteData() {
    while (!enable_.load() && run_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

#ifdef __linux__
    setProcessHighPriority(18);
// StickThisThreadToCore(5);
#endif  // __linux__

    if (enable_.load()) {
      data_file_ = std::ofstream(data_file_name_, std::ios::out);
      if (data_file_.bad()) {
        file_bad_ = true;
        logger_->error("failed to open {}", data_file_name_);
        return;
      }

      WriteTitle(headers_);

      std::vector<Number> temp;

      while (run_) {
        while (data_queue_.try_dequeue(temp)) {
          if (size_t data_size = temp.size(); data_size != headers_size_) {
            if (data_size > headers_size_) {
              logger_->warn("data logger warning: data_size > headers_size");
            } else {
              logger_->warn("data logger warning: data_size < headers_size");
            }
          }
          std::string str = DataToCSV(temp);
          data_file_.write(str.data(), str.size());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      while (data_queue_.try_dequeue(temp)) {
        if (size_t data_size = temp.size(); data_size != headers_size_) {
          if (data_size > headers_size_) {
            logger_->warn("data logger warning: data_size > headers_size");
          } else {
            logger_->warn("data logger warning: data_size < headers_size");
          }
        }
        std::string str = DataToCSV(temp);
        data_file_.write(str.data(), str.size());
      }
    }
  }

  struct NumberVisitor {
    std::string operator()(double& n) { return std::format("{:.6f}", n); }
    template <class T>
    std::string operator()(T& n) {
      return std::format("{}", n);
    }
  };

  std::string data_file_name_;
  std::string events_file_name_;
  std::vector<std::string> headers_;
  std::ofstream data_file_;
  bool file_bad_ = false;
  size_t headers_size_;
  moodycamel::ReaderWriterQueue<std::vector<Number>> data_queue_;
  bool run_ = true;

  std::atomic<bool> enable_ = false;
  std::thread write_thread_;

  Logger::Console logger_;
};

}  // namespace bitbot

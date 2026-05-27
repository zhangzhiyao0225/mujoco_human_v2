#ifndef CSV_DDS_LOGGER_HPP
#define CSV_DDS_LOGGER_HPP

#include <readerwriterqueue.h>

#include <atomic>
#include <format>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include "dds/handop_ui.hpp"

namespace ovinf {

class CsvDdsLogger {
 public:
  using Number = std::variant<uint8_t, uint16_t, uint32_t, float, int64_t,
                              uint64_t, double>;
  using Ptr = std::shared_ptr<CsvDdsLogger>;

  CsvDdsLogger(handop::RobotNotificationPublisher::Ptr publisher,
               const std::vector<std::string>& headers)
      : publisher_(publisher), data_queue_(1024) {
    WriteTitle(headers);
    run_.store(true);
    write_thread_ = std::thread(&CsvDdsLogger::WriteFile, this);
  }

  ~CsvDdsLogger() {
    run_.store(false);
    write_thread_.join();
  }

  void Write(std::vector<Number> const& data) {
    // if (data.size() != headers_size_) {
    //   if (data.size() > headers_size_) {
    //     std::cerr << "Data size is larger than header size" << std::endl;
    //   } else {
    //     std::cerr << "Data size is smaller than header size" << std::endl;
    //   }
    // }

    data_queue_.enqueue(data);
  }

 private:
  /**
   * @brief Thread funtcion
   *
   */
  void WriteFile() {
    std::vector<Number> temp;

    // std::cout << std::this_thread::get_id() << ": Start writing file " << run_
    //           << std::endl;

    while (run_) {
      while (data_queue_.try_dequeue(temp)) {
        RobotNotification noti;
        noti.data(std::move(DataToJson(temp)));
        publisher_->Publish(std::move(noti));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    while (data_queue_.try_dequeue(temp)) {
      RobotNotification noti;
      noti.data(std::move(DataToJson(temp)));
      publisher_->Publish(std::move(noti));
    }
  }

  void WriteTitle(const std::vector<std::string>& headers) {
    std::string title;

    std::for_each(headers.begin(), headers.end() - 1,
                  [&title](const std::string& header) {
                    std::format_to(std::back_inserter(title), "{},", header);
                  });
    std::format_to(std::back_inserter(title), "{}\n", headers.back());

    // file_.write(title.data(), title.size());
  }

  std::string DataToCSV(const std::vector<Number>& data) {
    static NumberVisitor visitor;
    std::string csv_str;

    std::for_each(data.begin(), data.end() - 1,
                  [&csv_str](const Number& number) {
                    std::format_to(std::back_inserter(csv_str), "{},",
                                   std::visit(visitor, number));
                  });
    std::format_to(std::back_inserter(csv_str), "{}\n",
                   std::visit(visitor, data.back()));

    return csv_str;
  }

  std::string DataToJson(const std::vector<Number>& data) {
    static NumberVisitor visitor;
    std::string json_str("{\"data\":[");

    std::for_each(data.begin(), data.end() - 1,
                  [&json_str](const Number& number) {
                    std::format_to(std::back_inserter(json_str), "{},",
                                   std::visit(visitor, number));
                  });
    std::format_to(std::back_inserter(json_str), "{}]",
                   std::visit(visitor, data.back()));
    json_str.append("}");

    return json_str;
  }

  struct NumberVisitor {
    std::string operator()(double& n) { return std::format("{:.6f}", n); }
    template <class T>
    std::string operator()(T& n) {
      return std::format("{}", n);
    }
  };

 private:
  handop::RobotNotificationPublisher::Ptr publisher_;
  moodycamel::ReaderWriterQueue<std::vector<Number>> data_queue_;
  std::thread write_thread_;

  std::atomic_bool run_ = true;
  // size_t headers_size_ = 0;
};

}  // namespace ovinf

#endif  // !CSV_LOGGER_HPP

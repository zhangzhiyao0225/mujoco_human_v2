#pragma once

#include "spdlog/spdlog.h"

namespace bitbot
{
  using SpdLoggerSharedPtr = std::shared_ptr<spdlog::logger>;

  struct LoggerImpl;

  class Logger 
  {
  public:
    Logger();

    /**
     * @brief 终端输出用的logger
     * 
     * @return SpdLoggerSharedPtr 
     */
    SpdLoggerSharedPtr ConsoleLogger();

    void SetConsoleLoggerLevel(spdlog::level::level_enum level);

    SpdLoggerSharedPtr CreateFileLogger(std::string file_name);

    using Console = SpdLoggerSharedPtr;
    using File = SpdLoggerSharedPtr;
  private:
    static LoggerImpl &impl();
    LoggerImpl &impl_;
  };

}

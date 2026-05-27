#include "utils/logger.h"

#include <ctime>
#include <filesystem>
#include <sstream>

#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace bitbot {
struct LoggerImpl {
  LoggerImpl() {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    // console_sink->set_level(spdlog::level::trace);
    console_sink->set_pattern("[%H:%M:%S.%e][%^%l%$] %v");

    console_logger = std::make_shared<spdlog::logger>(
        spdlog::logger("main_logger", {console_sink}));
    console_logger->set_level(spdlog::level::info);
    spdlog::register_logger(console_logger);
  }

  void SetConsoleLoggerLevel(spdlog::level::level_enum level) {
    console_logger->set_level(level);
  }

  SpdLoggerSharedPtr CreateFileLogger(const std::string& file_name) {
    // size_t max_size = 50 * 1024 * 1024;
    // size_t max_files = 20;
    // auto file_sink =
    // std::make_shared<spdlog::sinks::rotating_file_sink_mt>(file_name,
    // max_size, max_files, false);

    auto file_sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, false);

    file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

    SpdLoggerSharedPtr file_logger = std::make_shared<spdlog::logger>(
        spdlog::logger("file_logger_" + file_name, {file_sink}));

    // file_logger = spdlog::basic_logger_mt("bitbot_file_logger", file_name);
    // file_logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

    file_logger->set_level(spdlog::level::info);
    spdlog::register_logger(file_logger);

    return file_logger;
  }

  void FileloggerInit() {
    using namespace std::filesystem;

#ifdef _MSC_VER
    char home_path[256] = {0};
    size_t ret_size = 0;
    getenv_s(&ret_size, home_path, sizeof(home_path), "HOME");
#else
    char* home_path = getenv("HOME");
#endif  // _MSC_VER
    path home(home_path);

    path log_path(home / "bitbot_log" / "log");

    if (!exists(log_path)) {
      console_logger->info("Directory({}) does not exist. Now creating",
                           log_path.string());
      create_directories(log_path);
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
    tm* tm_buf = localtime(&now);
    std::string filetime =
        std::format("{:04d}{:02d}{:02d}_{:02d}{:02d}{:02d}",
                    tm_buf->tm_year + 1900, tm_buf->tm_mon + 1, tm_buf->tm_mday,
                    tm_buf->tm_hour, tm_buf->tm_min, tm_buf->tm_sec);
#endif

    CreateFileLogger((log_path / std::string(filetime + ".txt")).string());
  }

  SpdLoggerSharedPtr console_logger = nullptr;
};

Logger::Logger() : impl_(impl()) {}  // 构造函数直接初始化impl类

SpdLoggerSharedPtr Logger::ConsoleLogger() { return impl_.console_logger; }

void Logger::SetConsoleLoggerLevel(spdlog::level::level_enum level) {
  impl_.SetConsoleLoggerLevel(level);
}

SpdLoggerSharedPtr Logger::CreateFileLogger(std::string file_name) {
  return impl_.CreateFileLogger(file_name);
}

LoggerImpl& Logger::impl() {
  static LoggerImpl inst;
  return inst;
}

}  // namespace bitbot
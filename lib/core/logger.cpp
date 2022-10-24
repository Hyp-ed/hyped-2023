#include "logger.hpp"

#include <chrono>

namespace hyped::core {

Logger::Logger(const char *const label, const LogLevel level, const core::ITimeSource &timer)
    : label_(label),
      level_(level),
      timer_(timer)
{
}

void Logger::printHead(FILE *file, const char *title)
{
  const auto time_point = timer_.now();
  const auto ttime_t = std::chrono::system_clock::to_time_t(time_point);
  const auto tp_seconds = std::chrono::system_clock::from_time_t(ttime_t);
  const std::chrono::milliseconds tp_milliseconds = duration_cast<std::chrono::milliseconds>(time_point - tp_seconds);
  const std::tm *time_struct = localtime(&ttime_t);
  fprintf(file, "%02d:%02d:%02d.%03lld %s[%s] ", time_struct->tm_hour, time_struct->tm_min, time_struct->tm_sec, tp_milliseconds.count(), title, label_);
};

void Logger::log(const LogLevel level, const char *format, ...){
  FILE *file;
  if (level_ <= level) {
    switch (level) {
      case LogLevel::kDebug:
        file = stdout;
        printHead(file, "DEBUG");
        break;
      case LogLevel::kInfo:
        file = stdout;
        printHead(file, "INFO");
        break;
      case LogLevel::kFatal:
        file = stderr;
        printHead(file, "FATAL");
        break;
      default:
        break;
    }
    va_list args;
    va_start(args, format);
    vfprintf(file, format, args);
    fprintf(file, "\n");
    va_end(args);
  }
}

};  // namespace hyped::utils
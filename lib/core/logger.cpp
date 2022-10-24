#include "logger.hpp"

#include <chrono>

namespace hyped::utils {

Logger::Logger(const char *const module, const Level level, const core::ITimeSource &timer)
    : module_(module),
      level_(level),
      timer_(timer)
{
}

void Logger::intToLevel(const int level)
{
  switch (level) {
    case 0:
      level_ = Level::kNone;
      break;
    case 1:
      level_ = Level::kDebug;
      break;
    case 2:
      level_ = Level::kInfo;
      break;
    case 3:
      level_ = Level::kWarn;
      break;
    case 4:
      level_ = Level::kFatal;
      break;
    default:
      level_ = Level::kNone;
      break;
  }
};

void Logger::printHead(FILE *file, const char *title)
{
  const auto time_point = timer_.now();
  const auto ttime_t = std::chrono::system_clock::to_time_t(time_point);
  const auto tp_seconds = std::chrono::system_clock::from_time_t(ttime_t);
  const std::chrono::milliseconds tp_milliseconds = duration_cast<std::chrono::milliseconds>(time_point - tp_seconds);
  const std::tm *time_struct = localtime(&ttime_t);
  fprintf(file, "%02d:%02d:%02d.%03lld %s[%s] ", time_struct->tm_hour, time_struct->tm_min, time_struct->tm_sec, tp_milliseconds.count(), title, module_);
};

void Logger::print(FILE *file, const char *format, va_list args)
{
  vfprintf(file, format, args);
  fprintf(file, "\n");
};

void Logger::debug(const char *format, ...)
{
  FILE *file = stdout;
  if (level_ == Level::kDebug) {
    printHead(file, "DEBUG");
    va_list args;
    va_start(args, format);
    print(file, format, args);
    va_end(args);
  }
};

void Logger::info(const char *format, ...)
{
  FILE *file = stdout;
  if (level_ == Level::kInfo || level_ == Level::kDebug) {
    printHead(file, "INFO");
    va_list args;
    va_start(args, format);
    print(file, format, args);
    va_end(args);
  }
};

void Logger::warn(const char *format, ...)
{
  FILE *file = stderr;
  if (level_ == Level::kWarn || level_ == Level::kInfo || level_ == Level::kDebug) {
    printHead(file, "WARN");
    va_list args;
    va_start(args, format);
    print(file, format, args);
    va_end(args);
  }
};

void Logger::fatal(const char *format, ...)
{
  FILE *file = stderr;
  if (level_ == Level::kFatal || level_ == Level::kWarn || level_ == Level::kInfo || level_ == Level::kDebug) {
    printHead(file, "FATAL");
    va_list args;
    va_start(args, format);
    print(file, format, args);
    va_end(args);
  }
};

};  // namespace hyped::utils
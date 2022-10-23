#include "logger.hpp"

#include <chrono>

namespace hyped::utils {
void Logger::setLevel(const Level level)
{
  level_ = level;
};

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
  std::time_t t = std::time(nullptr);
  tm *tt        = localtime(&t);
  fprintf(file, "%02d:%02d:%02d", tt->tm_hour, tt->tm_min, tt->tm_sec);

  static const bool print_micro = true;
  if (print_micro) {
    auto now_time = std::chrono::high_resolution_clock::now().time_since_epoch();
    std::chrono::duration<int, std::milli> time_span
      = duration_cast<std::chrono::milliseconds>(now_time);
    fprintf(file, ".%03d ", static_cast<uint16_t>(time_span.count()) % 1000);
  } else {
    fprintf(file, " ");
  }
  fprintf(file, "%s[%s]: ", title, module_);
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
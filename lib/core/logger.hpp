#pragma once

#include <iostream>
#include <optional>
#include <string>

#include "time.hpp"

namespace hyped::utils {

enum class Level { kNone, kDebug, kInfo, kWarn, kFatal };

class ILogger {
 public:
  virtual void debug(const char *format, ...) = 0;
  virtual void info(const char *format, ...)  = 0;
  virtual void warn(const char *format, ...)  = 0;
  virtual void fatal(const char *format, ...) = 0;
};

class Logger : public ILogger {
 public:
  Logger(const char *const module, const Level level, const core::ITimeSource &timer);

  void intToLevel(const int level);
  void setLevel(const Level level);

  void printHead(FILE *file, const char *title);
  void print(FILE *file, const char *format, va_list args);

  void debug(const char *format, ...);
  void info(const char *format, ...);
  void warn(const char *format, ...);
  void fatal(const char *format, ...);

  private:
    const char *const module_;
    Level level_;
    const core::ITimeSource &timer_;
};
}  // namespace hyped::utils

#pragma once

#include <iostream>
#include <optional>
#include <string>

#include "time.hpp"

namespace hyped::core {

enum class LogLevel { kNone = 0, kDebug, kInfo, kFatal };

class ILogger {
 public:
  virtual void log(const LogLevel level, const char *format ...){};
};

class Logger : public ILogger {
 public:
  Logger(const char *const module, const LogLevel level, const core::ITimeSource &timer);

  void log(const LogLevel level, const char *format, ...);

  private:
    void printHead(FILE *file, const char *title);
    const char *const label_;
    LogLevel level_;
    const core::ITimeSource &timer_;
};
}  // namespace hyped::utils

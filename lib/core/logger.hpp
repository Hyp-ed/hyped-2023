#pragma once

#include "time.hpp"

#include <cstdarg>
#include <iostream>
#include <optional>

namespace hyped::core {

enum class LogLevel { kNone = 0, kDebug, kInfo, kFatal };

class ILogger {
 public:
  virtual void log(const LogLevel level, const char *format...) = 0;
};

class Logger : public ILogger {
 public:
  Logger(const char *const label, const LogLevel level, const core::ITimeSource &timer);

  void log(const LogLevel level, const char *format, ...);

 private:
  void printHead(FILE *file, const char *title);
  const char *const label_;
  const LogLevel level_;
  const core::ITimeSource &time_source_;
};
}  // namespace hyped::core

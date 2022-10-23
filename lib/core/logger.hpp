#pragma once

#include <iostream>
#include <optional>
#include <string>

namespace hyped::utils {

enum class Level { kNone, kDebug, kInfo, kWarn, kError, kFatal };

class ILogger {
 public:
  ILogger(const char *const module, const Level level) : module_(module), level_(level){};
  ILogger(const char *const module) : module_(module), level_(Level::kInfo){};
  ILogger() : module_("Logger"), level_(Level::kInfo){};

  void setLevel(const Level level);

  virtual void debug(const char *format, ...) = 0;
  virtual void info(const char *format, ...)  = 0;
  virtual void warn(const char *format, ...)  = 0;
  virtual void fatal(const char *format, ...) = 0;

 protected:
  const char *const module_;
  Level level_;
};

class Logger : public ILogger {
 public:
  Logger(const char *const module, const Level level) : ILogger(module, level){};
  Logger(const char *const module) : ILogger(module){};
  Logger() : ILogger(){};

  void intToLevel(const int level);
  void setLevel(const Level level);

  void printHead(FILE *file, const char *title);
  void print(FILE *file, const char *format, va_list args);

  void debug(const char *format, ...);
  void info(const char *format, ...);
  void warn(const char *format, ...);
  void fatal(const char *format, ...);
};
}  // namespace hyped::utils

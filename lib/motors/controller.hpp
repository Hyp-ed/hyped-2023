#pragma once
#include <cstdint>

#include "core/logger.hpp"

namespace hyped::motors {
enum class controller_status { ControllerTemperatureExceeded, GeneralWarning, Nominal };

class Controller {
 public:
  Controller(core::ILogger &logger);
  void processErrorMessage(const uint16_t error_code);
  controller_status processWarningMessage(const uint8_t warning_code);

 private:
  hyped::core::ILogger &logger_;
};

}  // namespace hyped::motors
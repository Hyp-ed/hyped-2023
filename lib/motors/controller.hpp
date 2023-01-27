#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <linux/can.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/istreamwrapper.h>

#include "core/logger.hpp"
#include "core/types.hpp"

namespace hyped::motors {
enum class ControllerStatus { kControllerTemperatureExceeded, kUnrecoverableWarning, kNominal };

class Controller {
 public:
  Controller(core::ILogger &logger);
  core::Result parseMessageFile(const std::string &path);
  void processErrorMessage(const std::uint16_t error_code);
  ControllerStatus processWarningMessage(const std::uint8_t warning_code);

 private:
  std::optional<core::CanFrame> parseJsonCanFrame(
    rapidjson::GenericObject<false, rapidjson::Value> message);
  core::ILogger &logger_;
  // TODO replace core::CanFrame with io::CanFrame once merged
  std::unordered_map<std::string, core::CanFrame> messages_;
  std::vector<core::CanFrame> configuration_messages_;
};

}  // namespace hyped::motors
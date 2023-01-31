#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/istreamwrapper.h>

#include "core/logger.hpp"
#include "core/types.hpp"

namespace hyped::motors {
enum class ControllerStatus { kControllerTemperatureExceeded, kUnrecoverableWarning, kNominal };
// every frame sent to the controller contains 8 bytes
static constexpr std::uint8_t kControllerCanFrameLength = 8;

class Controller {
 public:
  static std::optional<Controller> create(core::ILogger &logger,
                                          const std::string &message_file_path);
  void processErrorMessage(const std::uint16_t error_code);
  ControllerStatus processWarningMessage(const std::uint8_t warning_code);
  static std::optional<core::CanFrame> parseJsonCanFrame(
    rapidjson::GenericObject<false, rapidjson::Value> message, core::ILogger &logger);

 private:
  Controller(core::ILogger &logger,
             const std::unordered_map<std::string, core::CanFrame> messages,
             const std::vector<core::CanFrame> configuration_messages);
  core::ILogger &logger_;
  // TODO replace core::CanFrame with io::CanFrame once merged
  const std::unordered_map<std::string, core::CanFrame> messages_;
  const std::vector<core::CanFrame> configuration_messages_;
};

}  // namespace hyped::motors
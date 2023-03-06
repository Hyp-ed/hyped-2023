#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/istreamwrapper.h>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/can.hpp>

namespace hyped::motors {
// every frame sent to the controller contains 8 bytes
constexpr std::uint8_t kControllerCanFrameLength = 8;
enum class ControllerStatus { kControllerTemperatureExceeded, kUnrecoverableWarning, kNominal };

class Controller {
 public:
  static std::optional<Controller> create(core::ILogger &logger,
                                          const std::string &message_file_path);
  void processErrorMessage(const std::uint16_t error_code);
  ControllerStatus processWarningMessage(const std::uint8_t warning_code);
  static std::optional<io::CanFrame> parseJsonCanFrame(
    core::ILogger &logger, rapidjson::GenericObject<true, rapidjson::Value> message);

 private:
  Controller(core::ILogger &logger,
             const std::unordered_map<std::string, io::CanFrame> &messages,
             const std::vector<io::CanFrame> &configuration_messages);

 private:
  core::ILogger &logger_;
  // TODOLater replace io::CanFrame with io::CanFrame once merged
  const std::unordered_map<std::string, io::CanFrame> messages_;
  const std::vector<io::CanFrame> configuration_messages_;
};

}  // namespace hyped::motors
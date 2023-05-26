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
enum class ControllerStatus { kUnrecoverableWarning, kNominal };
enum class ControllerState {
  kOperationalState,
  kPreOperationalState,
  kStopState,
  kResetNodeState,
  kResetCommunicationState,
  kUnknownState
};

class Controller {
 public:
  static std::optional<Controller> create(core::ILogger &logger,
                                          const std::string &message_file_path);
  static std::optional<io::CanFrame> parseJsonCanFrame(
    core::ILogger &logger, rapidjson::GenericObject<true, rapidjson::Value> message);
  void processErrorMessage(const std::uint16_t error_code);
  ControllerStatus processWarningMessage(const std::uint8_t warning_code);
  core::Result processNmtMessage(const std::uint8_t nmt_code);
  core::Result processSdoMessage(const std::uint16_t index,
                                 const std::uint8_t subindex,
                                 std::uint32_t data);

 private:
  Controller(core::ILogger &logger,
             const std::unordered_map<std::string, io::CanFrame> &messages,
             const std::vector<io::CanFrame> &configuration_messages);
  std::uint8_t getControllerTemperature() const;
  core::Float getControllerCurrent() const;

 private:
  static constexpr std::uint16_t kSdoErrorIndex       = 0x603f;
  static constexpr std::uint16_t kSdoWarningIndex     = 0x2027;
  static constexpr std::uint16_t kSdoTemperatureIndex = 0x2026;
  static constexpr std::uint16_t kSdoCurrentIndex     = 0x2023;
  std::uint8_t controller_temperature_;
  core::Float controller_current_;
  ControllerState controller_state_;
  core::ILogger &logger_;
  // TODOLater replace io::CanFrame with io::CanFrame once merged
  const std::unordered_map<std::string, io::CanFrame> messages_;
  const std::vector<io::CanFrame> configuration_messages_;
};

}  // namespace hyped::motors
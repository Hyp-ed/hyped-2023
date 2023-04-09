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
#include <io/hardware_can.hpp>
namespace hyped::motors {
// every frame sent to the controller contains 8 bytes
constexpr std::uint8_t kControllerCanFrameLength = 8;
enum class ControllerStatus { kControllerTemperatureExceeded, kUnrecoverableWarning, kNominal };
constexpr std::uint16_t kControllerSdoSend         = 0x600;
constexpr std::uint16_t kControllerSdoReceive      = 0x580;
constexpr std::uint16_t kControllerSdoReadCommand  = 0x40;
constexpr std::uint16_t kControllerSdoWriteCommand = 0x23;

class Controller {
 public:
  static std::optional<Controller> create(core::ILogger &logger,
                                          const std::string &message_file_path,
                                          const std::shared_ptr<io::ICan> can);
  void processErrorMessage(const std::uint16_t error_code);
  ControllerStatus processWarningMessage(const std::uint8_t warning_code);
  static std::optional<io::CanFrame> parseJsonCanFrame(
    core::ILogger &logger, rapidjson::GenericObject<true, rapidjson::Value> message);
  /**
   * @brief Configures the motor controller with the configuration messages defined in the JSON file
   * of messages
   *
   * @return core::Result kSuccess if the configuration was successful, kFailure if the
   * configuration failed
   */
  core::Result configureController();

 private:
  Controller(core::ILogger &logger,
             const std::unordered_map<std::string, io::CanFrame> &messages,
             const std::vector<io::CanFrame> &configuration_messages);

 private:
  core::ILogger &logger_;
  const std::unordered_map<std::string, io::CanFrame> messages_;
  const std::vector<io::CanFrame> configuration_messages_;
  std::shared_ptr<io::ICan> can_;
};

}  // namespace hyped::motors
#pragma once

#include "frequency_calculator.hpp"

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
constexpr std::uint16_t kControllerSdoSend         = 0x601;
constexpr std::uint16_t kControllerSdoReceive      = 0x580;
constexpr std::uint16_t kControllerSdoReadCommand  = 0x40;
constexpr std::uint16_t kControllerSdoWriteCommand = 0x2B;
/**
 * @brief Temporary enum to represent the state of the state machine, will be replaced by the actual
 * state machine states
 *
 */
enum class FauxState { kInitial, kConfigure, kReady, kAccelerate, kStop, kReset };

class Controller {
 public:
  static std::optional<std::shared_ptr<Controller>> create(
    core::ILogger &logger,
    const std::string &message_file_path,
    const std::shared_ptr<io::ICan> can,
    const std::shared_ptr<IFrequencyCalculator> frequency_calculator);
  void processErrorMessage(const std::uint16_t error_code);
  ControllerStatus processWarningMessage(const std::uint8_t warning_code);
  static std::optional<io::CanFrame> parseJsonCanFrame(
    core::ILogger &logger, rapidjson::GenericObject<true, rapidjson::Value> message);

  /**
   * @brief Runs the motor controller in the provided state from state machine
   *
   * @param state the state to run the motor controller in
   * @return core::Result
   */
  core::Result run(FauxState state);

  Controller(core::ILogger &logger,
             const std::unordered_map<std::string, io::CanFrame> &messages,
             const std::vector<io::CanFrame> &configuration_messages,
             const std::shared_ptr<io::ICan> can,
             const std::shared_ptr<IFrequencyCalculator> frequency_calculator);

 private:
  /**
   * @brief Configures the motor controller with the configuration messages defined in the JSON file
   * of messages
   *
   * @return core::Result kSuccess if the configuration was successful, kFailure if the
   * configuration failed
   */
  core::Result configure();

  /**
   * @brief Causes the motor controller to accelerate the motor according to the curve defined by
   * frequency_calculator_
   *
   * @return core::Result kSuccess if the acceleration was successful, kFailure if the acceleration
   * failed
   */
  core::Result accelerate();

  /**
   * @brief Stops the motor controller
   *
   * @return core::Result kSuccess if the stop was successful, kFailure if the stop failed
   */
  core::Result stop();

  /**
   * @brief Resets of the motor controller
   *
   * @return core::Result kSuccess if the reset was successful, kFailure if the reset failed
   */
  core::Result reset();

 private:
  core::ILogger &logger_;
  const std::unordered_map<std::string, io::CanFrame> messages_;
  const std::vector<io::CanFrame> configuration_messages_;
  std::shared_ptr<io::ICan> can_;
  std::shared_ptr<IFrequencyCalculator> frequency_calculator_;
  // TODOLater work out how this is actually retrieved from navigation
  core::Float velocity_;
};

}  // namespace hyped::motors
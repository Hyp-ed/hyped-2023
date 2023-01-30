#include "controller.hpp"

#include <fstream>

namespace hyped::motors {

std::optional<Controller> Controller::create(core::ILogger &logger,
                                             const std::string &message_file_path)
{
  std::ifstream input_stream(message_file_path);
  if (!input_stream.is_open()) {
    logger.log(core::LogLevel::kFatal, "Failed to open file %s", message_file_path.c_str());
    return std::nullopt;
  }
  rapidjson::IStreamWrapper input_stream_wrapper(input_stream);
  rapidjson::Document document;
  const rapidjson::ParseResult result = document.ParseStream(input_stream_wrapper);
  if (!result) {
    logger.log(core::LogLevel::kFatal,
               "Error parsing JSON: %s",
               rapidjson::GetParseError_En(document.GetParseError()));
    return std::nullopt;
  }
  if (!document.HasMember("config_messages")) {
    logger.log(core::LogLevel::kFatal,
               "Missing required field 'config_messages' in can message file at %s",
               message_file_path.c_str());
    return std::nullopt;
  }
  if (!document.HasMember("messages")) {
    logger.log(core::LogLevel::kFatal,
               "Missing required field 'messages' in can message file at %s",
               message_file_path.c_str());
    return std::nullopt;
  }
  const auto configuration_messages = document["config_messages"].GetArray();
  std::vector<core::CanFrame> controller_configuration_messages;
  for (auto &message : configuration_messages) {
    const auto new_message = Controller::parseJsonCanFrame(message.GetObject(), logger);
    if (!new_message) {
      logger.log(core::LogLevel::kFatal,
                 "Invalid CAN configuration frame in JSON message file at path %s",
                 message_file_path.c_str());
      return std::nullopt;
    }
    controller_configuration_messages.push_back(*new_message);
  }
  std::unordered_map<std::string, core::CanFrame> controller_messages;
  const auto messages = document["messages"].GetObject();
  for (auto &message : messages) {
    const auto new_message = Controller::parseJsonCanFrame(message.value.GetObject(), logger);
    if (!new_message) {
      logger.log(core::LogLevel::kFatal,
                 "Invalid CAN frame in JSON message file at path %s",
                 message_file_path.c_str());
      return std::nullopt;
    }
    controller_messages.emplace(message.name.GetString(), *new_message);
  }
  return Controller(logger, controller_messages, controller_configuration_messages);
}

Controller::Controller(core::ILogger &logger,
                       const std::unordered_map<std::string, core::CanFrame> messages,
                       const std::vector<core::CanFrame> configuration_messages)
    : logger_(logger),
      configuration_messages_(configuration_messages),
      messages_(messages)
{
}

std::optional<core::CanFrame> Controller::parseJsonCanFrame(
  rapidjson::GenericObject<false, rapidjson::Value> message, core::ILogger &logger)
{
  if (!message.HasMember("id")) {
    logger.log(core::LogLevel::kFatal,
               "Missing required field 'id' in message in CAN message file");
    return std::nullopt;
  }
  if (!message.HasMember("index")) {
    logger.log(core::LogLevel::kFatal,
               "Missing required field 'index' in message in CAN message file");
    return std::nullopt;
  }
  if (!message.HasMember("subindex")) {
    logger.log(core::LogLevel::kFatal,
               "Missing required field 'subindex' in message in CAN message file");
    return std::nullopt;
  }
  if (!message.HasMember("data")) {
    logger.log(core::LogLevel::kFatal,
               "Missing required field 'data' in message in CAN message file");
    return std::nullopt;
  }
  core::CanFrame new_message;
  new_message.can_id  = message["id"].GetInt();
  new_message.can_dlc = 8;
  // TODO sort endianness conversion for index and data
  //  newmessage.data[0&1] = index
  new_message.data[2] = message["subindex"].GetInt();
  new_message.data[3] = 0;  // padding
  // newmessage.data[4-7] = index
  return new_message;
}

void Controller::processErrorMessage(const std::uint16_t error_code)
{
  switch (error_code) {
    case 0xFF01:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_CURRENT_A: Current phase A hall sensor missing or damaged");
      break;
    case 0xFF02:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_CURRENT_B: Current phase A hall sensor missing or damaged");
      break;
    case 0xFF03:
      logger_.log(core::LogLevel::kFatal, "ERROR_HS_FET: High side Fet short circuit");
      break;
    case 0xFF04:
      logger_.log(core::LogLevel::kFatal, "ERROR_LS_FET: Low side Fet short circuit");
      break;
    case 0xFF05:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_LS_L1: Low side Fet phase 1 short circuit");
      break;
    case 0xFF06:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_LS_L2: Low side Fet phase 2 short circuit");
      break;
    case 0xFF07:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_LS_L3: Low side Fet phase 3 short circuit");
      break;
    case 0xFF08:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_HS_L1: High side Fet phase 1 short circuit");
      break;
    case 0xFF09:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_HS_L2: High side Fet phase 2 short circuit");
      break;
    case 0xFF0A:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_HS_L3: High side Fet phase 3 short circuit ");
      break;
    case 0xFF0B:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_MOTOR_FEEDBACK: Wrong feedback selected (check feedback type)");
      break;
    case 0xFF0C:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_DC_LINK_UNDERVOLTAGE: DC voltage not applied to bridge or too low");
      break;
    case 0xFF0D:
      logger_.log(core::LogLevel::kFatal, "ERROR_PULS_MODE_FINISHED: Pulse mode finished");
      break;
    case 0xFF0E:
      logger_.log(core::LogLevel::kFatal, "ERROR_APP_ERROR");
      break;
    case 0xFF0F:
      logger_.log(core::LogLevel::kFatal, "ERROR_EMERGENCY_BUTTON: Emergency button pressed");
      break;
    case 0xFF10:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_CONTROLLER_OVERTEMPERATURE: Controller overtemperature");
      break;
    case 0x3210:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_DC_LINK_OVERVOLTAGE: Power supply voltage too high");
      break;
    default:
      logger_.log(core::LogLevel::kFatal,
                  "GENERIC_ERROR: Unspecific error occurred with code %i",
                  error_code);
      break;
  }
}

ControllerStatus Controller::processWarningMessage(const std::uint8_t warning_code)
{
  ControllerStatus priority_error = ControllerStatus::kNominal;

  // In the event norminal warning found, return.
  if (warning_code == 0) { return priority_error; }
  logger_.log(core::LogLevel::kInfo, "Controller Warning found, (code: %x)", warning_code);

  // In the event some warning(s) have occured, print each and return highest priority.
  if (warning_code & 0x1) {
    logger_.log(core::LogLevel::kInfo, "Controller Warning: Controller Temperature Exceeded");
    priority_error = ControllerStatus::kControllerTemperatureExceeded;
  }
  if (warning_code & 0x2) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: Motor Temperature Exceeded");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x4) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: DC link under voltage");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x8) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: DC link over voltage");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x10) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: DC link over current");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x20) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: Stall protection active");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x40) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: Max velocity exceeded");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x80) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: BMS Proposed Power");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x100) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: Capacitor temperature exceeded");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x200) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: I2T protection");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x400) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: Field weakening active");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  return priority_error;
}
}  // namespace hyped::motors
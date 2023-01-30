#include "bms.hpp"

namespace hyped::sensors {
Bms::Bms(core::ILogger &logger, io::HardwareCan &can) : logger_(logger), can_(can)
{
}

std::optional<BmsData> Bms::receiveBmsData()
{
  // TODO: Implement
  // Outline of possible implementation:
  BmsData bms_data;
  const auto optional_cell_temperatures = receiveCellTemperature();
  if (!optional_cell_temperatures) {
    logger_.log(core::LogLevel::kFatal, "failed to receive cell temperatures");
  }
  bms_data.temperatures = optional_cell_temperatures.value();
  // return bms_data;
  return std::nullopt;
}

std::optional<std::array<core::Float, NUM_BMS_CELLS>> Bms::receiveCellTemperature()
{
  // TODO: Implement
  // Outline of possible implementation:
  const auto bmsProcessor = std::make_shared<BmsProcessor>(logger_);
  can_.addProcessor(static_cast<std::uint16_t>(BmsPacketIds::kBmsStatusTemperatures), bmsProcessor);
  const auto frame = can_.receive();
  if (!frame) { logger_.log(core::LogLevel::kFatal, "Failed to receive cell temperatures"); }
  // would then parse frame and return temperatures
  return std::nullopt;
}

std::optional<io::CanFrame> Bms::receiveCellVoltage()
{
  // TODO: Implement
  return std::nullopt;
}

std::optional<io::CanFrame> Bms::receivePackCurrent()
{
  // TODO: Implement
  return std::nullopt;
}

}  // namespace hyped::sensors
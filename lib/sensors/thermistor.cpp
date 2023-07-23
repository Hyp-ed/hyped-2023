#include "thermistor.hpp"

namespace hyped::sensors {
std::optional<Thermistor> create(core::ILogger &logger,
                                 std::shared_ptr<io::IAdc> adc,
                                 std::uint8_t channel)
{
  return std::optional<Thermistor>(Thermistor(logger, adc, channel));
}

Thermistor::Thermistor(core::ILogger &logger, std::shared_ptr<io::IAdc> adc, std::uint8_t channel)
    : logger_(logger),
      adc_(adc),
      channel_(channel)
{
}

Thermistor::~Thermistor()
{
}

std::uint8_t Thermistor::getChannel() const
{
  return channel_;
}

std::optional<core::Float> Thermistor::read()
{
  const auto value = adc_->readValue();
  if (!value) {
    logger_.log(core::LogLevel::kFatal, "Failed to read thermistor value");
    return std::nullopt;
  }
  return std::optional<core::Float>(*value);
}

}  // namespace hyped::sensors

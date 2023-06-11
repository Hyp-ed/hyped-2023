#include <pressure.hpp>

namespace hyped::sensors {

Pressure::Pressure(core::ILogger &logger, std::shared_ptr<io::IAdc> adc)
    : logger_(logger),
      adc_(adc)
{
}

std::optional<core::Float> Pressure::read()
{
  const auto pressure = adc_->readValue();
  if (!pressure) {
    logger_.log(core::LogLevel::kFatal, "Failed to read pressure from ADC");
    return std::nullopt;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully read pressure from ADC");
  // Equation determined from testing
  return 6.944 * (*pressure) - 2.5;
}

}  // namespace hyped::sensors
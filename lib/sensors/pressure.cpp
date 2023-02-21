#include "pressure.hpp"

namespace hyped::sensors {
std::optional<SuspensionPressure> SuspensionPressure::create(core::ILogger &logger,
                                                             std::shared_ptr<io::IAdc> adc)
{
  logger.log(core::LogLevel::kDebug, "Successfully created SuspensionPressure instance");
  return SuspensionPressure(logger, adc);
}

SuspensionPressure::SuspensionPressure(core::ILogger &logger, std::shared_ptr<io::IAdc> adc)
    : adc_(adc),
      logger_(logger)
{
}

SuspensionPressure::~SuspensionPressure()
{
}

std::optional<std::uint8_t> SuspensionPressure::getPressure()
{
  const auto pressure_result = adc_->readValue();
  if (!pressure_result) {
    logger_.log(core::LogLevel::kFatal, "Failue for SuspensionPressure sensor to read adc value");
  }

  return *pressure_result;
}
}  // namespace hyped::sensors
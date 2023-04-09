#include "constant_frequency_calculator.hpp"

namespace hyped::motors {

ConstantFrequencyCalculator::ConstantFrequencyCalculator(core::ILogger &logger) : logger_(logger)
{
}

std::uint16_t ConstantFrequencyCalculator::calculateFrequency(core::Float frequency)
{
  return static_cast<std::uint16_t>(frequency);
}

}  // namespace hyped::motors
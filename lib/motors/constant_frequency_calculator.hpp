#pragma once

#include "frequency_calculator.hpp"

#include <cstdint>

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::motors {

class ConstantFrequencyCalculator : public IFrequencyCalculator {
 public:
  ConstantFrequencyCalculator(core::ILogger &logger);
  /**
   * @brief Returns the passed in velocity as the frequency
   *
   * @param frequency
   * @return core::Float equal to the passed in frequency
   */
  std::uint16_t calculateFrequency(core::Float frequency);

 private:
  core::ILogger &logger_;
};

}  // namespace hyped::motors

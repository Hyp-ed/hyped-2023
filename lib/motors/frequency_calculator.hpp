#pragma once

#include <cstdint>

#include <core/types.hpp>

namespace hyped::motors {

class IFrequencyCalculator {
 public:
  virtual std::uint16_t calculateFrequency(core::Float velocity) = 0;
};

}  // namespace hyped::motors
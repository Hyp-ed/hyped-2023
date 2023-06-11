#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/adc.hpp>

namespace hyped::sensors {
class Pressure {
 public:
  Pressure(core::ILogger &logger, std::shared_ptr<io::IAdc> adc);

  std::optional<core::Float> read();

 private:
  core::ILogger &logger_;
  std::shared_ptr<io::IAdc> adc_;
};

}  // namespace hyped::sensors
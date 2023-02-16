#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <io/adc.hpp>

namespace hyped::sensors {
class Pressure {
 public:
  static std::optional<Pressure> create(core::ILogger &logger,
                                        io::IAdc &adc);
  ~Pressure();

  std::uint8_t getPressure();

 private:
  Pressure(core::ILogger &logger, std::shared_ptr<io::IAdc> adc);

 private:
  std::shared_ptr<io::IAdc> adc_;
  core::ILogger &logger_;
};

}  // namespace hyped::sensors
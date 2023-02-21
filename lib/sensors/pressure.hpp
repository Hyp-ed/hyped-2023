#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <io/adc.hpp>

namespace hyped::sensors {
// TODOLater: test with hardware once we have it
class SuspensionPressure {
 public:
  static std::optional<SuspensionPressure> create(core::ILogger &logger,
                                                  std::shared_ptr<io::IAdc> adc);
  ~SuspensionPressure();

  std::optional<std::uint8_t> getPressure();

 private:
  SuspensionPressure(core::ILogger &logger, std::shared_ptr<io::IAdc> adc);

 private:
  std::shared_ptr<io::IAdc> adc_;
  core::ILogger &logger_;
};
}  // namespace hyped::sensors
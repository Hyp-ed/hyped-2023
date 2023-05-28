#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/adc.hpp>

namespace hyped::sensors {
class WheelEncoder {
 public:
  WheelEncoder(core::ILogger &logger, std::shared_ptr<io::IAdc> adc);
  std::uint64_t getCount();
  core::Result updateCount();
  void resetCount();

 private:
  static constexpr core::Float kVoltageThreshold = 1.7;
  core::ILogger &logger_;
  std::shared_ptr<io::IAdc> adc_;
  std::uint64_t count_;
  core::Float previous_voltage_;
};
}  // namespace hyped::sensors
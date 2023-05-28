#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/adc.hpp>

namespace hyped::sensors {
class WheelEncoder {
 public:
  std::uint64_t getCount();

  core::Result updateCount();

 private:
  WheelEncoder(core::ILogger &logger, std::shared_ptr<io::IAdc> adc);

 private:
  static constexpr core::Float kVoltageThreshold = 1.7;
  core::ILogger &logger_;
  std::shared_ptr<io::IAdc> adc_;
  std::uint64_t count_;
};
}  // namespace hyped::sensors
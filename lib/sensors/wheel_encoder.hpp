#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/adc.hpp>

namespace hyped::sensors {
class WheelEncoder {
 public:
  std::optional<WheelEncoder> create(core::ILogger &logger, std::shared_ptr<io::IAdc> adc);

  std::uint64_t getCount();

  void updateCount();

 private:
  WheelEncoder(core::ILogger &logger, std::shared_ptr<io::IAdc> adc);

 private:
  core::ILogger &logger_;
  std::shared_ptr<io::IAdc> adc_;
  const std::uint64_t count_;
};
}  // namespace hyped::sensors
#include <cstdint>
#include <vector>

#include <core/logger.hpp>
#include <core/time.hpp>
#include <core/timer.hpp>
#include <core/types.hpp>
#include <navigation/consts.hpp>

namespace hyped::utils {
class JerkAndSnap {
 public:
  JerkAndSnap(core::ILogger &logger, const core::ITimeSource &time);

  void updateAccelerationValues(const std::uint64_t timestamp, const core::Float acceleration);

  navigation::JerkSnap getJerkAndSnap();

 private:
  // TODO: decide 50 or 100 - numerical tests?
  static constexpr std::uint8_t kNumAccelerationValues = 50;
  std::vector<core::Float> acceleration_values;
  std::vector<std::uint64_t> timestamps;
};
}  // namespace hyped::utils
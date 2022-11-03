#include <array>
#include <cstdint>

#include "core/types.hpp"

namespace hyped::navigation {
class PreprocessImus {
 public:
  PreprocessImus();

  core::ImuData processData(const core::RawImuData raw_imu_data);

 private:
  core::ImuData imuOutlierDetection(const core::RawImuData imu_data);

  void checkImusReliable(const core::ImuData imu_data);

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumImus> outlier_imus_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumImus> reliable_imus_;
};

}  // namespace hyped::navigation
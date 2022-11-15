#include "consts.hpp"

#include <array>
#include <cstdint>

#include "core/types.hpp"

namespace hyped::navigation {
class ImuPreprocessor {
 public:
  ImuPreprocessor();

  core::ImuData processData(const core::RawImuData raw_imu_data);

 private:
  core::ImuData detectOutliers(const core::RawImuData imu_data);

  void checkReliable();
  
  template<std::size_t N> Quartiles getQuartiles(const std::array<core::Float, N> clean_accelerometer_data_copy);

  Quartiles getOutlierThresholds(const core::ImuData &imu_data);


  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumImus> num_outliers_per_imu_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumImus> are_imus_reliable_;

  // TODO: describe this
  static constexpr std::uint8_t kNumAllowedImuFailures_ = 20;
};

}  // namespace hyped::navigation
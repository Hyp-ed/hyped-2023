#include "consts.hpp"

#include <cmath>

#include <algorithm>
#include <array>
#include <cstdint>

#include "core/logger.hpp"
#include "core/types.hpp"

namespace hyped::navigation {
class ImuPreprocessor {
 public:
  ImuPreprocessor(core::ILogger &logger);

  std::optional<core::ImuData> processData(const core::RawImuData raw_imu_data);

 private:
  core::ImuData detectOutliers(const core::ImuData imu_data);

  SensorChecks checkReliable();

  template<std::size_t N>
  core::Float getSpecificQuantile(const std::array<core::Float, N> &clean_accelerometer_data,
                                  core::Float quartile)
  {
    const core::Float index_quartile       = (num_reliable_accelerometers_ - 1) * quartile;
    const std::uint8_t index_quartile_high = static_cast<int>(std::ceil(index_quartile));
    const std::uint8_t index_quartile_low  = static_cast<int>(std::floor(index_quartile));
    const core::Float quartile_value       = (clean_accelerometer_data.at(index_quartile_high)
                                        + clean_accelerometer_data.at(index_quartile_low))
                                       / 2.0;
    return quartile_value;
  }

  template<std::size_t N>
  Quartiles getQuartiles(const std::array<core::Float, N> &accelerometer_data)
  {
    std::array<core::Float, N> accelerometer_data_copy;
    std::copy(
      accelerometer_data.begin(), accelerometer_data.end(), accelerometer_data_copy.begin());

    std::sort(accelerometer_data_copy.begin(), accelerometer_data_copy.end());
    Quartiles quartiles;
    return {quartiles.q1     = getSpecificQuantile(accelerometer_data_copy, 0.25),
            quartiles.median = getSpecificQuantile(accelerometer_data_copy, 0.5),
            quartiles.q3     = getSpecificQuantile(accelerometer_data_copy, 0.75)};
  }

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumImus> num_outliers_per_imu_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumImus> are_imus_reliable_;

  // number of allowed consecutive outliers from single accelerometer
  static constexpr std::uint8_t kNumAllowedImuFailures_ = 20;

  std::size_t num_reliable_accelerometers_;  // intitialised as= core::kNumImus

  core::ILogger &logger_;
};

}  // namespace hyped::navigation
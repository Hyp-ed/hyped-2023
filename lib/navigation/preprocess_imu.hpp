#pragma once

#include "consts.hpp"

#include <cmath>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

#include "core/logger.hpp"
#include "core/types.hpp"

namespace hyped::navigation {
class ImuPreprocessor {
 public:
  ImuPreprocessor(core::ILogger &logger);

  /**
   * @brief convert raw imu data to cleaned and filtered data
   *
   * @param raw_imu_data
   * @return clean imu data or optionally fail
   */
  std::optional<core::ImuData> processData(const core::RawImuData raw_imu_data);

 private:
  core::ILogger &logger_;

  std::array<std::uint16_t, core::kNumImus> num_outliers_per_imu_;

  std::array<bool, core::kNumImus> are_imus_reliable_;

  std::size_t num_reliable_accelerometers_;

  // number of allowed consecutive outliers from single accelerometer
  static constexpr std::uint8_t kNumAllowedImuFailures_ = 20;

  /**
   * @brief filter the imu data by converting outliers to median value
   *
   * @param imu_data
   * @return filtered imu data
   */
  core::ImuData detectOutliers(const core::ImuData imu_data);

  /**
   * @brief check the reliability of all imu's
   *
   * @return SensorChecks with value kAcceptable or kUnacceptable
   */
  SensorChecks checkReliable();

  // TODOLater: Optimize this further
  /**
   * @brief find value at a specific quantile of the data
   *
   * @param clean_accelerometer_data
   * @param quartile
   * @return value at specified quantile
   */
  template<std::size_t N>
  core::Float getSpecificQuantile(const std::array<core::Float, N> &clean_accelerometer_data,
                                  core::Float quartile)
  {
    const core::Float index_quartile = (num_reliable_accelerometers_ - 1) * quartile;
    const int index_quartile_high    = static_cast<int>(std::ceil(index_quartile));
    const int index_quartile_low     = static_cast<int>(std::floor(index_quartile));
    const core::Float quartile_value = (clean_accelerometer_data.at(index_quartile_high)
                                        + clean_accelerometer_data.at(index_quartile_low))
                                       / 2.0;
    return quartile_value;
  }

  /**
   * @brief find values at each quartile of the data
   *
   * @param accelerometer_data
   * @return values of each quartile
   */
  template<std::size_t N>
  Quartiles getQuartiles(const std::array<core::Float, N> &accelerometer_data)
  {
    std::array<core::Float, N> accelerometer_data_copy;
    std::copy(
      accelerometer_data.begin(), accelerometer_data.end(), accelerometer_data_copy.begin());

    std::sort(accelerometer_data_copy.begin(), accelerometer_data_copy.end());
    return {.q1     = getSpecificQuantile(accelerometer_data_copy, 0.25),
            .median = getSpecificQuantile(accelerometer_data_copy, 0.5),
            .q3     = getSpecificQuantile(accelerometer_data_copy, 0.75)};
  }
};

}  // namespace hyped::navigation

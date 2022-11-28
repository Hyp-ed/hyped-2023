#include "consts.hpp"

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
  core::Float getSpecificQuartile(const std::array<core::Float, N> &clean_accelerometer_data_copy,
                                  core::Float quartile);

  template<std::size_t N>
  Quartiles getQuartiles(const std::array<core::Float, N> &imu_data);

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumImus> num_outliers_per_imu_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumImus> are_imus_reliable_;

  // number of allowed consecutive outliers from single accelerometer
  static constexpr std::uint8_t kNumAllowedImuFailures_ = 20;

  std::size_t num_reliable_accelerometers_;  // intitialised as= core::kNumImus

  core::ILogger &log_;
};

}  // namespace hyped::navigation
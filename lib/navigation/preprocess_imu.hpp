#pragma once

#include "types.hpp"

#include <array>
#include <cstdint>

#include "core/types.hpp"

namespace hyped::navigation {

class ImuPreprocessor {
 public:
  ImuPreprocessor();

  std::optional<ImuData> processData(const core::RawAccelerationData &raw_acceleration_data);

 private:
  ImuData detectOutliers(const core::RawAccelerationData &raw_acceleration_data);

  void checkReliable(const ImuData &imu_data);

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<std::uint16_t, core::kNumImus> num_outliers_per_imu_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumImus> are_imus_reliable_;
};

}  // namespace hyped::navigation

#include "navigator.hpp"

#include <cstdint>

namespace hyped::navigation {

Navigator::Navigator()
{
  // TODO: impement
}

void Navigator::navigate()
{
  /*
  TODO: implement

  main navigation control:

  - look at new data
  - preprocess
  - check sensor agreement
  - calculate current trajecotory
  - update current trajectory
  */
}

void Navigator::recordImuData(const core::RawImuData imu_data)
{
  for (std::size_t i = 0; i < core::kNumImus; ++i) {
    raw_imu_data_.at(i) = imu_data.at(i);
  }
}

void Navigator::recordEncoderData(const core::EncoderData encoder_data)
{
  raw_encoder_data_ = encoder_data;
}

void Navigator::recordKeyenceData(const core::KeyenceData keyence_data)
{
  raw_keyence_data_ = keyence_data;
}

}  // namespace hyped::navigation

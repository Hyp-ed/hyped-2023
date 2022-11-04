#include "navigator.hpp"

#include <cstdint>

namespace hyped::navigation {

Navigator::Navigator()
{
  // TODOLater: impement
}

void Navigator::navigate()
{
  /*
  TODOLater: implement

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
}

void Navigator::recordEncoderData(const core::EncoderData encoder_data)
{
}

void Navigator::recordKeyenceData(const core::KeyenceData keyence_data)
{
}

}  // namespace hyped::navigation

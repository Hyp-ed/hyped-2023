#include "data.hpp"

namespace hyped::navigation::benchmark {

DataBuilder::DataBuilder()
{
}

Data DataBuilder::getData()
{
  return data_;
}

core::Result DataBuilder::addEncoderData(const core::TimePoint &timestamp,
                                         const core::RawEncoderData &encoder_data)
{
  const auto was_insertion_successful
    = data_.encoder_data_by_time.emplace(timestamp, encoder_data).second;
  if (was_insertion_successful) {
    return core::Result::kSuccess;
  } else {
    return core::Result::kFailure;
  }
}

core::Result DataBuilder::addEncoderData(const std::uint64_t seconds_since_epoch,
                                         const core::RawEncoderData &encoder_data)
{
  const auto timestamp
    = std::chrono::system_clock::time_point(std::chrono::seconds(seconds_since_epoch));
  return addEncoderData(timestamp, encoder_data);
}

core::Result DataBuilder::addAccelerationData(const core::TimePoint &timestamp,
                                              const core::RawAccelerationData &acceleration_data)
{
  const auto was_insertion_successful
    = data_.acceleration_data_by_time.emplace(timestamp, acceleration_data).second;
  if (was_insertion_successful) {
    return core::Result::kSuccess;
  } else {
    return core::Result::kFailure;
  }
}

core::Result DataBuilder::addAccelerationData(const std::uint64_t seconds_since_epoch,
                                              const core::RawAccelerationData &acceleration_data)
{
  const auto timestamp
    = std::chrono::system_clock::time_point(std::chrono::seconds(seconds_since_epoch));
  return addAccelerationData(timestamp, acceleration_data);
}

core::Result DataBuilder::addKeyenceData(const core::TimePoint &timestamp,
                                         const core::RawKeyenceData &keyence_data)
{
  const auto was_insertion_successful
    = data_.keyence_data_by_time.emplace(timestamp, keyence_data).second;
  if (was_insertion_successful) {
    return core::Result::kSuccess;
  } else {
    return core::Result::kFailure;
  }
}

core::Result DataBuilder::addKeyenceData(const std::uint64_t seconds_since_epoch,
                                         const core::RawKeyenceData &keyence_data)
{
  const auto timestamp
    = std::chrono::system_clock::time_point(std::chrono::seconds(seconds_since_epoch));
  return addKeyenceData(timestamp, keyence_data);
}

}  // namespace hyped::navigation::benchmark

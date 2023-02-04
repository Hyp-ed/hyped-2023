#include "data.hpp"

#include <algorithm>

#include "core/types.hpp"

namespace hyped::navigation::benchmark {

std::vector<core::TimePoint> Data::getRelevantTimes() const
{
  std::vector<core::TimePoint> times(encoder_data_by_time.size() + acceleration_data_by_time.size()
                                     + keyence_data_by_time.size()
                                     + trajectory_data_by_time.size());
  for (const auto &[time_point, _] : encoder_data_by_time) {
    times.push_back(time_point);
  }
  for (const auto &[time_point, _] : acceleration_data_by_time) {
    times.push_back(time_point);
  }
  for (const auto &[time_point, _] : keyence_data_by_time) {
    times.push_back(time_point);
  }
  for (const auto &[time_point, _] : trajectory_data_by_time) {
    times.push_back(time_point);
  }
  std::sort(times.begin(), times.end());
  return times;
}

std::optional<core::RawEncoderData> Data::getEncoderDataAt(const core::TimePoint time_point) const
{
  const auto it = encoder_data_by_time.find(time_point);
  if (it == encoder_data_by_time.end()) { return std::nullopt; }
  return it->second;
}

std::optional<core::RawAccelerometerData> Data::getAccelerationDataAt(
  const core::TimePoint time_point) const
{
  const auto it = acceleration_data_by_time.find(time_point);
  if (it == acceleration_data_by_time.end()) { return std::nullopt; }
  return it->second;
}

std::optional<core::RawKeyenceData> Data::getKeyenceDataAt(const core::TimePoint time_point) const
{
  const auto it = keyence_data_by_time.find(time_point);
  if (it == keyence_data_by_time.end()) { return std::nullopt; }
  return it->second;
}

std::optional<core::Trajectory> Data::getTrajectoryDataAt(const core::TimePoint time_point) const
{
  const auto it = trajectory_data_by_time.find(time_point);
  if (it == trajectory_data_by_time.end()) { return std::nullopt; }
  return it->second;
}

DataBuilder::DataBuilder()
{
}

Data DataBuilder::build()
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

core::Result DataBuilder::addUniformEncoderData(const std::uint64_t nanos_since_epoch,
                                                const std::uint32_t total_num_revolutions)
{
  const auto timestamp
    = std::chrono::system_clock::time_point(std::chrono::seconds(nanos_since_epoch));
  core::RawEncoderData raw_encoder_data;
  for (std::size_t i = 0; i < core::kNumEncoders; ++i) {
    raw_encoder_data.at(i) = total_num_revolutions;
  }
  return addEncoderData(timestamp, raw_encoder_data);
}

core::Result DataBuilder::addAccelerationData(const core::TimePoint &timestamp,
                                              const core::RawAccelerometerData &acceleration_data)
{
  const auto was_insertion_successful
    = data_.acceleration_data_by_time.emplace(timestamp, acceleration_data).second;
  if (was_insertion_successful) {
    return core::Result::kSuccess;
  } else {
    return core::Result::kFailure;
  }
}

core::Result DataBuilder::addUniformAccelerationData(
  const std::uint64_t nanos_since_epoch,
  const std::array<core::Float, core::kNumAxis> raw_acceleration)
{
  const auto timestamp = core::timePointFromNanosSinceEpoch(nanos_since_epoch);
  core::RawAccelerometerData raw_acceleration_data;
  for (std::size_t i = 0; i < core::kNumAccelerometers; ++i) {
    raw_acceleration_data.at(i) = raw_acceleration;
  }
  return addAccelerationData(timestamp, raw_acceleration_data);
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

core::Result DataBuilder::addKeyenceData(const std::uint64_t nanos_since_epoch,
                                         const core::RawKeyenceData &keyence_data)
{
  const auto timestamp = core::timePointFromNanosSinceEpoch(nanos_since_epoch);
  return addKeyenceData(timestamp, keyence_data);
}

core::Result DataBuilder::addTrajectoryData(const core::TimePoint &timestamp,
                                            const core::Trajectory &trajectory)
{
  const auto was_insertion_successful
    = data_.trajectory_data_by_time.emplace(timestamp, trajectory).second;
  if (was_insertion_successful) {
    return core::Result::kSuccess;
  } else {
    return core::Result::kFailure;
  }
}

core::Result DataBuilder::addTrajectoryData(const std::uint64_t nanos_since_epoch,
                                            const core::Trajectory &trajectory)
{
  const auto timestamp = core::timePointFromNanosSinceEpoch(nanos_since_epoch);
  return addTrajectoryData(timestamp, trajectory);
}

}  // namespace hyped::navigation::benchmark

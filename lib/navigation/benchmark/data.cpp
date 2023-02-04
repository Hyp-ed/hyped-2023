#include "data.hpp"

#include <algorithm>

#include <core/types.hpp>

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

std::optional<core::CombinedRawAccelerometerData> Data::getAccelerationDataAt(
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

core::Result DataBuilder::addEncoderData(const core::RawEncoderData &encoder_data)
{
  const auto it = data_.encoder_data_by_time.emplace(encoder_data.measured_at, encoder_data);
  if (!(it.second)) { return core::Result::kFailure; }
  return core::Result::kSuccess;
}

core::Result DataBuilder::addUniformEncoderData(const std::uint64_t nanos_since_epoch,
                                                const std::uint32_t total_num_revolutions)
{
  const auto measured_at
    = std::chrono::system_clock::time_point(std::chrono::seconds(nanos_since_epoch));
  std::array<std::uint32_t, core::kNumEncoders> num_revolutions_per_wheel;
  for (std::size_t i = 0; i < core::kNumEncoders; ++i) {
    num_revolutions_per_wheel.at(i) = total_num_revolutions;
  }
  return addEncoderData(core::Measurement(measured_at, num_revolutions_per_wheel));
}

core::Result DataBuilder::addAccelerationData(
  const core::CombinedRawAccelerometerData &acceleration_data)
{
  const auto it
    = data_.acceleration_data_by_time.emplace(acceleration_data.measured_at, acceleration_data);
  if (!(it.second)) { return core::Result::kFailure; }
  return core::Result::kSuccess;
}

core::Result DataBuilder::addUniformAccelerationData(const std::uint64_t nanos_since_epoch,
                                                     const core::RawAcceleration &raw_acceleration)
{
  const auto measured_at = core::timePointFromNanosSinceEpoch(nanos_since_epoch);
  std::array<core::RawAcceleration, core::kNumAccelerometers> raw_accelerometer_data;
  for (std::size_t i = 0; i < core::kNumAccelerometers; ++i) {
    raw_accelerometer_data.at(i) = raw_acceleration;
  }
  return addAccelerationData(
    core::CombinedRawAccelerometerData(measured_at, raw_accelerometer_data));
}

core::Result DataBuilder::addKeyenceData(const core::RawKeyenceData &keyence_data)
{
  const auto it = data_.keyence_data_by_time.emplace(keyence_data.measured_at, keyence_data);
  if (!(it.second)) { return core::Result::kFailure; }
  return core::Result::kSuccess;
}

core::Result DataBuilder::addTrajectoryData(const core::TimePoint timestamp,
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

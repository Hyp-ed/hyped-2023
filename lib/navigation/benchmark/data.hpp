#pragma once

#include <map>
#include <optional>
#include <vector>

#include <core/time.hpp>
#include <core/types.hpp>

namespace hyped::navigation::benchmark {

struct Data {
  std::map<core::TimePoint, core::RawEncoderData> encoder_data_by_time;
  std::map<core::TimePoint, core::RawAccelerationData> acceleration_data_by_time;
  std::map<core::TimePoint, core::RawKeyenceData> keyence_data_by_time;
  std::map<core::TimePoint, core::Trajectory> trajectory_data_by_time;

  std::vector<core::TimePoint> getRelevantTimes() const;
  std::optional<core::RawEncoderData> getEncoderDataAt(const core::TimePoint time_point) const;
  std::optional<core::RawAccelerationData> getAccelerationDataAt(
    const core::TimePoint time_point) const;
  std::optional<core::RawKeyenceData> getKeyenceDataAt(const core::TimePoint time_point) const;
  std::optional<core::Trajectory> getTrajectoryDataAt(const core::TimePoint time_point) const;
};

class DataBuilder {
 public:
  DataBuilder();
  Data build();

  core::Result addEncoderData(const core::TimePoint &timestamp,
                              const core::RawEncoderData &encoder_data);
  core::Result addEncoderData(const std::uint64_t seconds_since_epoch,
                              const core::RawEncoderData &encoder_data);
  core::Result addAccelerationData(const core::TimePoint &timestamp,
                                   const core::RawAccelerationData &acceleration_data);
  core::Result addAccelerationData(const std::uint64_t seconds_since_epoch,
                                   const core::RawAccelerationData &acceleration_data);
  core::Result addKeyenceData(const core::TimePoint &timestamp,
                              const core::RawKeyenceData &keyence_data);
  core::Result addKeyenceData(const std::uint64_t seconds_since_epoch,
                              const core::RawKeyenceData &keyence_data);
  core::Result addTrajectoryData(const core::TimePoint &timestamp,
                                 const core::Trajectory &trajectory);
  core::Result addTrajectoryData(const std::uint64_t seconds_since_epoch,
                                 const core::Trajectory &trajectory);

 private:
  Data data_;
};

}  // namespace hyped::navigation::benchmark

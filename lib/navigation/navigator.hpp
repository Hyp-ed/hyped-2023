#include "consts.hpp"

#include "core/types.hpp"

namespace hyped::navigation {

class Navigator : public INavigator {
 public:
  Navigator();

  /**
   *@brief runs cross checking and returns trajectory
   */
  core::Trajectory currentTrajectory();

  /**
   * @brief preprocesses keyence data and updates trajectory
   *
   * @param keyence_data
   */
  void keyenceUpdate(const core::KeyenceData &keyence_data);
  /**
   * @brief preprocesses encoder data and updates trajectory
   *
   * @param encoder_data
   */
  void encoderUpdate(const core::EncoderData &encoder_data);
  /**
   * @brief preprocesses imu data and updates trajectory
   *
   * @param imu_data
   */
  void imuUpdate(const core::RawImuData &imu_data);

 private:
  // previous readings
  core::EncoderData encoder_data_;
  core::KeyenceData keyence_data_;

  // current navigation trajectory
  core::Trajectory trajectory_;
};

}  // namespace hyped::navigation
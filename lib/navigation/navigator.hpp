#include "consts.hpp"

#include "core/types.hpp"

namespace hyped::navigation {

class Navigator {
 public:
  Navigator();

  /**
   *@brief Does navigation. Combines all navigation functionality to produce the best
   *current estimate of trajectory
   */
  void navigate();

  /**
   * @brief Set Imu Data from sensors
   *
   * @param imu_data
   */
  void setImuData(const core::RawImuData imu_data);

  /**
   * @brief Set Encoder Data from sensors
   *
   * @param encoder_data
   */
  void setEncoderData(const core::EncoderData encoder_data);

  /**
   * @brief Set Keyence Data from sensors
   *
   * @param keyence_data
   */
  void setKeyenceData(const core::KeyenceData keyence_data);

 private:
  core::RawImuData raw_imu_data_;
  core::EncoderData raw_encoder_data_;
  core::KeyenceData raw_keyence_data_;

  core::ImuData imu_data_;
  core::EncoderData encoder_data_;
  core::KeyenceData keyence_data_;

  // current navigation trajectory
  core::Trajectory trajectory_;

  // current keyence value for displacement
  int keyenceDisplacement_;
};
}  // namespace hyped::navigation

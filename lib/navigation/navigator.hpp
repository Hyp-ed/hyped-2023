#include "consts.hpp"
#include "core/types.hpp"

namespace hyped::navigation{

class Navigator{

  public:
  Navigator();

  /**
  *@brief Does navigation. Combines all navigation functionality to produce the best
  *current estimate of trajectory
  */
  void navigate();

  /**
  * @brief Publishes the current navigation trajectory to wherever it goes
  * TODO: make this more informative once implemented
  */
  void publishTrajectory();

  /**
   * @brief Set Imu Data from sensors
   * 
   * @param imu_data 
   */
  void setImuData(const core::ImuData imu_data);

  /**
   * @brief Set Encoder Data from sensors
   * 
   * @param encoder_data 
   */
  void setEncoderData(const core::WheelEncoderData encoder_data);

  /**
   * @brief Set Keyence Data from sensors
   * 
   * @param keyence_data 
   */
  void setKeyenceData(const core::KeyenceData keyence_data);

  private:

  std::array<nav_t, kNumImus> imu_data_;
  std::array<int32_t, kNumEncoders> encoder_data_;
  std::array<int32_t, kNumKeyence> keyence_data_;

  
  //current navigation trajectory
  Trajectory trajectory_;

  //current keyence value for displacement
  int keyenceDisplacement_;
};
}  // namespace hyped::navigation

#pragma once

#include "types.hpp"

#include <array>

#include <core/types.hpp>

namespace hyped::navigation {

class Crosscheck {
 public:
  Crosscheck();

  /**
   * @brief checks that all sensors agree with wheel encoders. This is a safety
   * check for the reliability of the current trajectory (i.e. the sensors are
   * probably not all wrong...)
   *
   * @return true signifies trajectory agreement
   * @return false signifies trajectory disagreement. We enter fail state
   */
  SensorDisagreement checkTrajectoryAgreement(const AccelerometerData accelerometer_data,
                                              const EncoderData encoders_data,
                                              const KeyenceData keyence_data);

 private:
  /**
   * @brief Checks the double integrated accelerometer value of displacement against
   * the encoder value of displacement to some tolerance
   * TODOLater: update comment with tolerance once updated
   *
   * @return true accelerometer and wheel encoders agree
   * @return false accelerometers and wheel encoders disagree
   */
  SensorDisagreement checkEncoderAccelerometer(const AccelerometerData accelerometer_data,
                                               const EncoderData encoders_data);

  /**
   * @brief Checks the keyence value of displacement against the
   * encoder value of displacement to some tolerance
   * TODOLater: update comment with tolerance once updated
   *
   * @return true Keyence and wheel encoders agree
   * @return false Keyence and wheel encoders disagree
   */
  SensorDisagreement checkEncoderKeyence(const EncoderData encoder_data,
                                         const KeyenceData keyence_data);
};
}  // namespace hyped::navigation

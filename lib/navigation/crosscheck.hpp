#include <array>

namespace hyped::navigation{

  class Crosscheck{

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
      bool checkTrajectoryAgreement();

    private:

      /**
       * @brief Checks the double integrated IMU value of displacement against
       * the encoder value of displacement to some tolerance
       * TODO: update comment with tolerance once updated
       * 
       * @return true IMU and wheel encoders agree
       * @return false IMU and wheel encoders disagree
       */
      bool checkEncoderImu();

      /**
       * @brief Checks the integrated camera value of displacement against the 
       * encoder value of displacement to some tolerance
       * TODO: update comment with tolerance once updated
       * 
       * @return true Camera and wheel encoders agree
       * @return false Camera and wheel encoders disagree
       */
      bool checkEncoderCamera();

      /**
       * @brief Checks the keyence value of displacement against the 
       * encoder value of displacement to some tolerance
       * TODO: update comment with tolerance once updated
       * 
       * @return true Keyence and wheel encoders agree
       * @return false Keyence and wheel encoders disagree
       */
      bool checkEncooderKeyence();

  };
}
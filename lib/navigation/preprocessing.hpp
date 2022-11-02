#include <array>
#include "consts.hpp"
#include "core/types.hpp"

namespace hyped::navigation{

  class Preprocessing{
    
    public:
    
    Preprocessing();

    /**
     * @brief runs outlier detection and sensor reliability on IMU data
     * TODO: update comment with what we do with data once implemented
     * 
     */
    core::ImuData preprocessImus(const core::RawImuData imu_data);

    /**
     * @brief runs outlier detection and sensor reliability on encoders data
     * TODO: update comment with what we do with data once implemented
     * 
     */
    core::EncoderData preprocessEncoders(const core::EncoderData encoder_data);

    /**
     * @brief checks that keyence data is  consistent
     * TODO: update comment with what we do with data once implemented
     * 
     */
    core::KeyenceData preprocessKeyence(const core::KeyenceData keyence_data);
    
    private:

    //raw sensor data
    core::RawImuData raw_imu_data_;
    core::EncoderData raw_encoder_data_;
    core::KeyenceData raw_keyence_data_;
    
    //processed sensor data
    core::ImuData imu_data_;
    core::EncoderData encoder_data_;
    core::KeyenceData keyence_data_;
    //TODO: Some form of camera data? Need to get camera idea going first.

    /**
     * @brief runs outlier detection on IMUs
     * TODO: update comment with what we do with data once implemented
     * 
     */
    void imuOutlierDetection();

    /**
     * @brief runs outlier detection on wheel encoders
     * TODO: update comment with what we do with data once implemented
     */
    void encoderOutlierDetection();

    /**
     * @brief checks that IMU data is still reliable
     * TODO: update comment with what we do with data once implemented
     */
    void checkImusReliable();

    /**
     * @brief checks that wheel encoders data is still reliable
     * TODO: update comment with what we do with data once implemented
     */
    void checkEncodersReliable();

    //arrays denoting which sensors are reliable
    std::array<bool, core::kNumImus> reliable_imus_;
    std::array<bool, core::kNumEncoders> reliable_encoders_;

    //arrays to keep track of how many times a sensor has been an outlier consecutively
    std::array<int, core::kNumImus> outlier_imus_ = {0, 0, 0, 0};
    std::array<int, core::kNumEncoders> oultier_encoders_ = {0, 0, 0, 0};

    //flag for if keyence sensors are in disagreement
    bool keyence_disagreement_;
  };
}
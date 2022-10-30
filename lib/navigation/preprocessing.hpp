#include <array>
#include "consts.hpp"

namespace hyped::navigation{

  class Preprocessing{
    
    public:
    
    Preprocessing();

    /**
     * @brief runs outlier detection and sensor reliability on IMU data
     * TODO: update comment with what we do with data once implemented
     * 
     */
    void preprocessImus();

    /**
     * @brief runs outlier detection and sensor reliability on encoders data
     * TODO: update comment with what we do with data once implemented
     * 
     */
    void preprocessEncoders();

    /**
     * @brief checks that keyence data is  consistent
     * TODO: update comment with what we do with data once implemented
     * 
     */
    void preprocessKeyence();

    /**
     * @brief Get the Imu Data object
     * 
     * @return std::array<nav_t, kNumImus> 
     */
    std::array<nav_t, kNumImus> getImuData();

    /**
     * @brief Get the Encoder Data object
     * 
     * @return std::array<int, kNumEncoders> 
     */
    std::array<int, kNumEncoders> getEncoderData();

    /**
     * @brief Get the Keyence Data object
     * 
     * @return std::array<int, kNumKeyence> 
     */
    std::array<int, kNumKeyence> getKeyenceData();
    
    private:
    
    //current data values for imus, encdoers and keyence
    std::array<nav_t, kNumImus> imu_data_;
    std::array<int, kNumEncoders> encoder_data_;
    std::array<int, kNumKeyence> keyence_data_;
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
    std::array<bool, kNumImus> reliable_imus_;
    std::array<bool, kNumEncoders> reliable_encoders_;

    //arrays to keep track of how many times a sensor has been an outlier consecutively
    std::array<int, kNumImus> outlier_imus_ = {0, 0, 0, 0};
    std::array<int, kNumEncoders> oultier_encoders_ = {0, 0, 0, 0};

    //flag for if keyence sensors are in disagreement
    bool keyence_disagreement_;
  };
}
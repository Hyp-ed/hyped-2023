#include <array>
#include "consts.hpp"

namespace hyped::navigation{

  class Preprocessing{
    
    public:
    
    Preprocessing();

    void preprocessImus();

    void preprocessEncoders();

    void preprocessKeyence();


    //TODO: make better types & set number of sensors as constant somewhere else
    std::array<nav_t, kNumImus> imu_data_;
    std::array<int, kNumEncoders> encoder_data_;
    std::array<int, kNumKeyence> keyence_data_;
    

    private:

    void imuOutlierDetection();

    void encoderOutlierDetection();


    void checkImusReliable();

    void checkEncodersReliable();


    //TODO: make better
    std::array<bool, kNumImus> reliable_imus_;
    std::array<bool, kNumEncoders> reliable_encoders_;

    std::array<int, kNumImus> outlier_imus_ = {0, 0, 0, 0};
    std::array<int, kNumEncoders> oultier_encoders_ = {0, 0, 0, 0};

    bool keyence_disagreement_;

  };
}
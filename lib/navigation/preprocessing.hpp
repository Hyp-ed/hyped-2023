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
    std::array<nav_t, kNumberImus> imu_data_;
    std::array<int, kNumberEncoders> encoder_data_;
    std::array<int, kNumberKeyence> keyence_data_;
    

    private:

    void imuOutlierDetection();

    void encoderOutlierDetection();


    void checkImusReliable();

    void checkEncodersReliable();


    //TODO: make better
    std::array<bool, kNumberImus> reliable_imus_;
    std::array<bool, kNumberEncoders> reliable_encoders_;

    std::array<int, kNumberImus> outlier_imus_ = {0, 0, 0, 0};
    std::array<int, kNumberEncoders> oultier_encoders_ = {0, 0, 0, 0};

    bool keyence_disagreement_;

  };
}
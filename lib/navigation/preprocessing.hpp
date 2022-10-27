#include <array>

namespace hyped::navigation{

  class Preprocessing{
    
    public:
    
    Preprocessing();

    void preprocessImus();

    void preprocessEncoders();

    void preprocessKeyence();


    //TODO: make better types & set number of sensors as constant somewhere else
    std::array<float, 4> imu_data_;
    std::array<int, 4> encoder_data_;
    std::array<int, 2> keyence_data_;
    

    private:

    void imuOutlierDetection();

    void encoderOutlierDetection();


    void checkImusReliable();

    void checkEncodersReliable();


    //TODO: make better
    std::array<bool, 4> reliable_imus_;
    std::array<bool, 4> reliable_encoders_;

    std::array<int, 4> outlier_imus_ = {0, 0, 0, 0};
    std::array<int, 4> oultier_encoders_ = {0, 0, 0, 0};

    bool keyence_disagreement_;

  };
}
#include "preprocessing.hpp"

namespace hyped::navigation{

  Preprocessing::Preprocessing(){
    //TODO: implement
  }

  void Preprocessing::imuOutlierDetection(){
    /*
    TODO: implement
    rough process:
    - get q1, median, q3 of imu array
    - define upper & lower bounds as (-)1.5*inter-quatrile range
    - if any datapoint outwith this, set 
    outlier_imus_[i] += 1
    -set outlier points as median

    -also has to be able to handle 1 unreliable sensor
    -also figure out return type/ what we update and update
    documentation as appropriate
    */
  }

  void Preprocessing::encoderOutlierDetection(){
    /*
    TODO: implement
    rough process:
    - get q1, median, q3 of encoder array
    - define upper & lower bounds as (-)1.5*inter-quatrile range
    - if any datapoint outwith this, set 
    outlier_imus_[i] += 1
    -set outlier points as median

    -also has to be able to handle 1 unreliable sensor
    -also figure out return type/ what we update and update
    documentation as appropriate
    */
  }

  void Preprocessing::checkImusReliable(){
    /*
    TODO: implement
    rough process:
    - check how many times an individual imu 
    has been an outlier in a row (outlier imus)
    -if outlier_imus_[i] > n (tbd), mark imu as unreliable 
    in reliable imus.
    -if number unreliable in reliable_imus > 1, fail state

    -also figure out return type/ what we update and update
    documentation as appropriate
    */
  }

  void Preprocessing::checkEncodersReliable(){
    /*
    TODO: implement
    rough process:
    - check how many times an individual encoder 
    has been an outlier in a row (outlier encoders)
    -if outlier_encoders_[i] > n (tbd), mark imu as unreliable 
    in reliable imus.
    -if number unreliable in reliable_imus > 1, fail state

    -also figure out return type/ what we update and update
    documentation as appropriate
    */
  }

  std::array<uint32_t, kNumKeyence> Preprocessing::preprocessKeyence(const std::array<uint32_t, kNumKeyence> keyence_data){
    /*
    TODO: implement
    rough process:
    - check if keyence sensors agree
    - if they agree, set keyence disgreeemnt to false
    - if they disagree and keyenece_disagreement = false, set to true
    - if they disagree and keyenece_disagreement = true, 
    we have 2 disgeements in a row so fail state
    - update with largest of sensor values
    
    -also figure out return type/ what we update and update
    documentation as appropriate
    */
   return {0, 0};
  }

  std::array<nav_t, kNumImus> Preprocessing::preprocessImus(const std::array<nav_t, kNumImus> imu_data){

    /*
    TODO: implement
    rough plan:

    - assign imu_data to class object
    - run imu outlier detection
    - run check imus reliable
    - get mean and run kalman filter
    - update trajectory with mean

    -also figure out return type/ what we update and update
    documentation as appropriate
    */
   return {0.0, 0.0, 0.0, 0.0};
  }

  std::array<uint32_t, kNumEncoders> Preprocessing::preprocessEncoders(const std::array<uint32_t, kNumEncoders> encoder_data){

    /*
    TODO: implement
    rough plan:

    - assign encoder data to class object
    - run encoder outlier detection
    - run check encoders reliable
    - update trajectory with mean

    -also figure out return type/ what we update and update
    documentation as appropriate
    */
   return {0, 0, 0, 0};
  }

  std::array<nav_t, kNumImus> Preprocessing::getImuData(){
    return imu_data_;
  }

  std::array<uint32_t, kNumEncoders> Preprocessing::getEncoderData(){
    return encoder_data_;
  }

  std::array<uint32_t, kNumKeyence> Preprocessing::getKeyenceData(){
    return keyence_data_;
  }

}
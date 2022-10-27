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

    */
  }

  void Preprocessing::preprocessKeyence(){
    /*
    TODO: implement
    rough process:
    - check if keyence sensors agree
    - if they agree, set keyence disgreeemnt to false
    - if they disagree and keyenece_disagreement = false, set to true
    - if they disagree and keyenece_disagreement = true, 
    we have 2 disgeements in a row so fail state
    */
  }

  void Preprocessing::preprocessImus(){

    /*
    TODO: implement
    rough plan:

    - check how we move data (should this be void, probably not)
    - run imu outlier detection
    - run check imus reliable
    */
  }


  void Preprocessing::preprocessEncoders(){

    /*
    TODO: implement
    rough plan:

    - check how we move data (should this be void, probably not)
    - run encoder outlier detection
    - run check encoders reliable
    */
  }


}
#include "crosscheck.hpp"

namespace hyped::navigation{

  Crosscheck::Crosscheck(){
    //TODO: implement
  }

  bool Crosscheck::checkTrajectoryAgreement(){
    /*
    TODO: implement
    basically:
      - checkEncoderImu
      - checkEncoderCamera
      - checkEncooderKeyence
      - if all good, return true. else false and fail state
    */
   return true;
  }

  bool checkEncoderImu(){
    /*
    TODO: implement.
    plan:
    - double integrate imu values (also todo)
    - if absolute diff between encoder displacement and
    imu displacement too high, fail state and return false
    - otherwise all good, return true
    */
   return true;
  }

  bool checkEncoderCamera(){
    /*
    TODO: implement.
    plan:
    - integrate camera values (also todo)
    - if absolute diff between encoder displacement and
    camera displacement too high, fail state and return false
    - otherwise all good, return true
    */
    return true;
  }

  bool checkEncoderKeyence(){
    /*
    TODO: implement.
    plan:
    - if absolute diff between encoder displacement and
    keyence displacement too high, fail state and return false
    - otherwise all good, return true
    */
   return true;
  }
}
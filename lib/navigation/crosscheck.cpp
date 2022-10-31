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
      - checkEncooderKeyence
      - if all good, return true. else false and fail state

      Also need to figure out how data flow is going to work with the historic data and what we use.
      The basic infrastrucutre is there for now so will be a problem for another day.
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
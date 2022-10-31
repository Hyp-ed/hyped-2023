#include "navigator.hpp"

namespace hyped::navigation {

Navigator::Navigator(){
  //TODO: impement
}

void Navigator::navigate(){
  /*
  TODO: implement

  main navigation control:

  - look at new data
  - preprocess
  - check sensor agreement
  - calculate current trajecotory
  - update current trajectory
  */
}

void Navigator::publishTrajectory(){
  /*
  TODO: implement

  idk how ros works but make sure everyone who needs to know trajectory has it
  */
}

void Navigator::setImuData(const core::ImuData imu_data){
  imu_data_.at(0) = static_cast<nav_t>(imu_data.imu0);
  imu_data_.at(1) = static_cast<nav_t>(imu_data.imu1);
  imu_data_.at(2) = static_cast<nav_t>(imu_data.imu2);
  imu_data_.at(3) = static_cast<nav_t>(imu_data.imu3);
}

void Navigator::setEncoderData(const core::WheelEncoderData encoder_data){
  encoder_data_.at(0) = encoder_data.encoder0;
  encoder_data_.at(1) = encoder_data.encoder1;
  encoder_data_.at(2) = encoder_data.encoder2;
  encoder_data_.at(3) = encoder_data.encoder3;
}

void Navigator::setKeyenceData(const core::KeyenceData keyence_data){
  keyence_data_.at(0) = keyence_data.keyence0;
  keyence_data_.at(1) = keyence_data.keyence1;
}

}  // namespace hyped::navigation

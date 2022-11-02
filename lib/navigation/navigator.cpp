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


void Navigator::setImuData(const core::RawImuData imu_data){
  for(size_t i = 0; i < core::kNumImus; ++i){
    raw_imu_data_.at(i) = imu_data.at(i);
  }
}

void Navigator::setEncoderData(const core::EncoderData encoder_data){
  for(size_t i = 0; i < core::kNumEncoders; ++i){
    raw_encoder_data_.at(i) = encoder_data.at(i);
  }
}

void Navigator::setKeyenceData(const core::KeyenceData keyence_data){
  for(size_t i = 0; i < core::kNumKeyence; ++i){
    raw_keyence_data_.at(i) = keyence_data.at(i);
  }
}

}  // namespace hyped::navigation

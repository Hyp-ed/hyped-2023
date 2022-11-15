#include "preprocess_encoders.hpp"
#include "core/types.hpp"
#include <algorithm>
#include <array>
#include <cmath>  

namespace hyped::navigation {
  //core::EncoderData reliability_of_encoders;
  /*Initialising the array with 1(assuming that all the encoders are initially reliable )
  a value of 0 corresponding at the ith position means that the encoder is unreliable 
  */
  //std::fill_n(reliability_of_encoders.begin(),core::kNumEncoders, 1);
  //std::fill_n(reliability_of_encoders.begin(),reliability_of_encoders.end(), 1); 
  //core::EncoderData num_outliers_per_encoder;

  //std::array<uint16_t, core::kNumEncoders> num_outliers_per_encoder_;
  // std::array<bool, core::kNumEncoders> are_encoders_reliable_;

EncodersPreprocessor::EncodersPreprocessor()
{
  // TODOLater: implement
}

core::EncoderData EncodersPreprocessor::processData(const core::EncoderData)
{
  /*
  TODOLater: implement
  basic plan:
  - call detectOutliers. Return of that function is return of this function
  - call checkRelaible

  */
  return {0, 0, 0, 0};
}

/*
core::EncoderData EncodersPreprocessor::detectOutliers(const core::EncoderData encoder_data)
{
  /*
  TODOLater: implement
  rough process:
  - get q1, median, q3 of encoder array
  - define upper & lower bounds as (-)1.5*inter-quatrile range
  - if any datapoint outwith this, set
  num_outliers_per_encoder_[i] += 1
  -set outlier points as median

  -also has to be able to handle 1 unreliable sensor
  -also figure out return type/ what we update and update
  documentation as appropriate
  }
  */
template<typename T>
std::array<core::Float,3> EncodersPreprocessor::quartiles(T encoder_data){
  core::Float median;
  core::Float q1;
  core::Float q3;
  std::array<core::Float,3> output;
  int q1_high = static_cast<int> (std::ceil((encoder_data.size()+1)/4.0));
  int q1_low = static_cast<int> (std::floor((encoder_data.size()+1)/4.0));
  int q3_high = static_cast<int> (std::ceil((3*(encoder_data.size()+1))/4.0));
  int q3_low = static_cast<int> (std::ceil((3*(encoder_data.size()+1))/4.0));
  q1 = encoder_data.at(q1_low-1) + (((encoder_data.size()+1)/4.0) - q1_low)*(encoder_data.at(q1_high-1) - encoder_data.at(q1_low-1));
  q3 = encoder_data.at(q3_high-1) +(((3*(encoder_data.size()+1))/4.0) - q3_low)*(encoder_data.at(q3_high-1) - encoder_data.at(q3_low-1));
  if(encoder_data.size() % 2 == 0){
    median = (encoder_data.at((encoder_data.size()/2)-1) + encoder_data.at(((encoder_data.size()/2) + 1))-1)/2.0;
  }
  else{
   median = encoder_data.at(((encoder_data.size()+1)/2)-1);
  }
  output.at(0) = median;
  output.at(1) = q1;
  output.at(2) = q3;

  return output; // return an output of the sequence median,q1,q3
}

core::EncoderData EncodersPreprocessor::detectOutliers(const core::EncoderData encoder_data){
  
 return {0, 0, 0, 0};
}



/*
void EncodersPreprocessor::checkReliable(const core::EncoderData &encoder_data)
{
  /*
  TODOLater: implement
  rough process:
  - check how many times an individual encoder
  has been an outlier in a row (outlier encoders)
  -if num_outliers_per_encoder_[i] > n (tbd), mark encoder as unreliable
  in reliable encoders.
  -if number unreliable in reliable_encoders > 1, fail state

  -also figure out return type/ what we update and update
  documentation as appropriate
  

}
*/

void EncodersPreprocessor::checkReliable(const core::EncoderData num_outliers_per_encoder){
  for(int i =0;i<core::kNumEncoders;i++){
    if(num_outliers_per_encoder.at(i) > 10){//for now assuming n to be 10
        are_encoders_reliable_.at(i) = false; //the encoder is now unrealiable
    }    
  }
}


}  // namespace hyped::navigation
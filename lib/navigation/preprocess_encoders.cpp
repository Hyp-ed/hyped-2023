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

  std::array<uint16_t, core::kNumEncoders> num_outliers_per_encoder_;
  std::array<bool, core::kNumEncoders> are_encoders_reliable_;

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
core::EncoderData EncodersPreprocessor::detectOutliers(const core::EncoderData encoder_data){
  //int sum = std::accumulate(encoder_data.begin(), encoder_data.end(), 0);
  int sum = 0;// could do this by std::accumulate,but idk why it dosen't work
  for(int i = 0;i<encoder_data.size();i++){
    sum  = sum + are_encoders_reliable_.at(i);
  }
  core::Float median;
  core::Float q1;
  core::Float q3;
  core::Float iqr;
  core::Float upper_bound;  // ask how to calculate upper bound
  core::Float lower_bound;   //ask how to calculate lower bound
  if(sum == 1){
    std::array<uint32_t,3> encoder_data_copy;
    core::EncoderData encoder_data_copy2;
    std::copy(encoder_data.begin(),encoder_data.end(),encoder_data_copy2.begin());
    int counter = 0;
    for(int i = 0;i<encoder_data.size();i++){
      if(are_encoders_reliable_.at(i) == 0){
        encoder_data_copy.at(counter) = encoder_data.at(i);
        counter = counter + 1;
      }

    }
    std::sort(encoder_data_copy.begin(), encoder_data_copy.end());
    median = encoder_data_copy.at(1);
    q1 = encoder_data_copy.at(0);
    q3 = encoder_data_copy.at(2);
    iqr = q3-q1;
    upper_bound = 0;//ask how to find them
    lower_bound = 0;//ask how to find them
    for(int i =0;i<encoder_data_copy.size();i++){
      if(encoder_data_copy.at(i) > upper_bound || encoder_data_copy.at(i) < lower_bound){//check if the condition is correct
        const unsigned int *index = std::find(encoder_data.begin(),encoder_data.end(), encoder_data_copy.at(i));//finding the index of the element in encoder's data
        encoder_data_copy.at(i) = median;
        num_outliers_per_encoder_.at(*index) = num_outliers_per_encoder_.at(*index) + 1;
        encoder_data_copy2.at(*index) = median;
      }
    }
    return encoder_data_copy2;
    
  }
  core::EncoderData encoder_data_copy;
  std::copy(encoder_data.begin(),encoder_data.end(),encoder_data_copy.begin());
  std::sort(encoder_data_copy.begin(), encoder_data_copy.end());
  median = (encoder_data_copy.at((std::ceil(encoder_data_copy.size()/2.0))) + encoder_data_copy.at((std::floor(encoder_data_copy.size()/2.0))))/2.0;
  q1 = (encoder_data_copy.at(0) + encoder_data_copy.at(1))/2.0;
  q3 = (encoder_data_copy.at(2) + encoder_data_copy.at(3))/2.0;
  iqr = q3-q1;
  upper_bound = 0;//ask how to find them
  lower_bound = 0;//ask how to find them
  for(int i =0;i<encoder_data_copy.size();i++){
      if(encoder_data_copy.at(i) > upper_bound || encoder_data_copy.at(i) < lower_bound){//check if the condition is correct
        const unsigned int *index = std::find(encoder_data.begin(),encoder_data.end(), encoder_data_copy.at(i));//finding the index of the element in encoder's data
        encoder_data_copy.at(i) = median;
        num_outliers_per_encoder_.at(*index) = num_outliers_per_encoder_.at(*index) + 1;
      }
    }
return encoder_data_copy;
 //return {0, 0, 0, 0};
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
        are_encoders_reliable_.at(i) = 1; //the encoder is now unrealiable
    }    
  }
}


}  // namespace hyped::navigation
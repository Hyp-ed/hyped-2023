#include "preprocess_encoders.hpp"
#include "core/types.hpp"
#include <iterator>
#include <algorithm>
#include <array>
#include <vector>
namespace hyped::navigation {

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
  */
 //core::EncoderData encoder_data_copy = std::copy(encoder_data.begin(), encoder_data.end(), std::back_inserter(encoder_data));
 //core::EncoderData encoder_data_copy = std::copy(encoder_data.begin(), encoder_data.end())
 core::EncoderData encoder_data_copy;
 for(int i = 0;i<core::kNumEncoders;i++){
  encoder_data_copy.at(i) = encoder_data.at(i);
 }
 core::Float median = calulate_value(encoder_data_copy);
 if(encoder_data.size() % 2 == 0){
    median =  ((encoder_data.at(encoder_data.size()/2) + encoder_data.at((encoder_data.size()/2) + 1))/2.0);
  }
  else{
    median = encoder_data.at(encoder_data.size() + 1)/2.0;
  }
 core::Float q1 = encoder_data_copy.at(((encoder_data_copy.size() + 1)/4) -1);
 core::Float q3 = encoder_data_copy.at((3*(encoder_data_copy.size() + 1)/4) -1;

 return {0, 0, 0, 0};
} 
core::EncoderData EncodersPreprocessor::detectOutliers(const core::EncoderData encoder_data){

}


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
  */
}

}  // namespace hyped::navigation
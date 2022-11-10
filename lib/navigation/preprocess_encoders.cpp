#include "preprocess_encoders.hpp"
#include "core/types.hpp"
#include <iterator>
#include <algorithm>
#include <array>

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
 core::Float median = 0;
 for(int i = 0;i<core::kNumEncoders;i++){
  encoder_data_copy.at(i) = encoder_data.at(i);
 }
 if(encoder_data.size() % 2 == 0){
    median =  ((encoder_data.at(encoder_data.size()/2) + encoder_data.at((encoder_data.size()/2) + 1))/2.0);  //calculating the median of encoder data if the length of the array is even
                                                                                                               
  }
  else{
    median = encoder_data.at(encoder_data.size() + 1)/2.0; //median if the length of encoder's data is odd
  }
 core::Float q1 = encoder_data_copy.at(((encoder_data_copy.size() + 1)/4) -1);    // Formula taken from net
 core::Float q3 = encoder_data_copy.at((3*(encoder_data_copy.size() + 1)/4) -1);  // Formula tken from net

 core::EncoderData num_outliers_per_encoder;
 for(int i =0;i<core::kNumEncoders;i++){
  if(encoder_data_copy.at(i) > q3 || encoder_data_copy.at(i) < q1){
    num_outliers_per_encoder.at(i) = num_outliers_per_encoder.at(i) + 1;
    encoder_data_copy.at(i) = median;   // updating the value of outlier to the median
  }
 }

 return {0, 0, 0, 0}; // might be a good idea to create an array of size 2 such that index 0 contains our updated encoder_data and the element at index 1 is an array of the number of outliers per encoder
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
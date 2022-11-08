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
 core::EncoderData encoder_data_copy = std::copy(encoder_data.begin(), encoder_data.end(), std::back_inserter(encoder_data));
 std::sort(encoder_data_copy.begin(), encoder_data_copy.end());
 core::Float median = 0;
 core::Float q1 = 0.0;
 core::Float q3 = 0.0;
 if(encoder_data_copy.size() % 2 == 0){
  median = (encoder_data_copy[(std::size(encoder_data_copy))/2] + encoder_data_copy[((std::size(encoder_data_copy))/2) + 1])/2.0;
 }
 else{
  median = encoder_data_copy[(std::size(encoder_data_copy) + 1)/2];
 }
 core::EncoderData array_for_q1 ;
 std::copy_if(encoder_data_copy.begin(),encoder_data_copy.end(),array_for_q1.begin(),[](uint32_t i){return i< median;});
 core::EncoderData array_for_q2;
 std::copy_if(encoder_data_copy.begin(),encoder_data_copy.end(),array_for_q2.begin(),[](uint32_t i){return i> median;});
 if(array_for_q1.size() % 2 == 0){
  q1 = (array_for_q1[(array_for_q1.size())/2] +array_for_q1[((array_for_q1.size())/2) + 1] )/2.0;
 }
 else{
  q1 = array_for_q1[(array_for_q1.size()+1)/2.0];
 }
 if(array_for_q2.size() % 2 == 0){
  q3 = (array_for_q2[(array_for_q2.size()/2)] + array_for_q2[((array_for_q2.size()/2) + 1)])/2.0;
 }
 else{
  q3 = array_for_q2[(array_for_q2.size() + 1)]/2.0;
 }

 //const core::Float q1 = (encoder_data_copy.at(0) + encoder_data_copy.at(1))/2.0;
 //const core::Float q3 = (encoder_data_copy.at(2) + encoder_data_copy.at(3))/2.0;
 const core::Float iqr = q3-q1;
 return {0, 0, 0, 0};
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
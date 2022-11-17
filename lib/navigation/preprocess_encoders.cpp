#include "preprocess_encoders.hpp"
#include "core/types.hpp"
#include <algorithm>
#include <array>
#include <cmath>  
#include <numeric>

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
 
template<typename T>
std::array<core::Float,3> EncodersPreprocessor::quartiles(T encoder_data){
  core::Float median;
  core::Float q1;
  core::Float q3;
  core::Float iqr;
  core::Float upper_bound;
  core::Float lower_bound;
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
  iqr = q3-q1;
  upper_bound = median + 1.5*(iqr);
  lower_bound = median - 1.5*(iqr);
  output.at(0) = median;
  output.at(1) = upper_bound;
  output.at(2) = lower_bound;

  return output; // return an output of the sequence median,upper_bound,lower_bound
}*/

template<std::size_t N>
Quartile EncodersPreprocessor::getQuartiles(std::array<std::uint32_t , N> & encoder_data){
  core::Float median;
  core::Float q1;
  core::Float q3;
  core::Float iqr;
  core::Float upper_bound;
  core::Float lower_bound;
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
  iqr = q3-q1;
  upper_bound = median + 1.5*(iqr);
  lower_bound = median - 1.5*(iqr);
  struct Quartile quartile;
  quartile.median = median;
  quartile.upperBound = upper_bound;
  quartile.lowerBound = lower_bound;

  return quartile; // return an output of the sequence median,upper_bound,lower_bound
}


/*core::EncoderData EncodersPreprocessor::detectOutliers(const core::EncoderData encoder_data){
  const uint8_t num_reliable_encoders = std::accumulate(are_encoders_reliable_.begin(),are_encoders_reliable_.end(),0);
  core::Float median;
  core::Float upper_bound;
  core::Float lower_bound;
  core::EncoderData encoder_data_copy;
  std::copy(encoder_data.begin(),encoder_data.end(),encoder_data_copy.begin());
  if(num_reliable_encoders == 3){
    std::array<uint32_t,3> encoder_data_copy_for_finding_quartiles;
    int counter = 0;
    for(int i = 0;i<encoder_data.size();++i){
      if(are_encoders_reliable_.at(i) == true){
        encoder_data_copy_for_finding_quartiles.at(counter) = encoder_data.at(i);
        counter = counter + 1;
      }
    }
    std::sort(encoder_data_copy_for_finding_quartiles.begin(),encoder_data_copy_for_finding_quartiles.end());
    std::array<core::Float,3> array_of_quartiles = quartiles(encoder_data_copy_for_finding_quartiles); // ask how to pass the value
    median = array_of_quartiles.at(0);
    upper_bound = array_of_quartiles.at(1);
    lower_bound = array_of_quartiles.at(2);
  }
  else{
    core::EncoderData encoder_data_copy_for_finding_quartiles;
    std::copy(encoder_data.begin(),encoder_data.end(),encoder_data_copy_for_finding_quartiles.begin());
    std::sort(encoder_data_copy_for_finding_quartiles.begin(),encoder_data_copy_for_finding_quartiles.end());
    std::array<core::Float,3> array_of_quartiles = quartiles(encoder_data_copy_for_finding_quartiles); // ask how to pass the value
    median = array_of_quartiles.at(0);
    upper_bound = array_of_quartiles.at(1);
    lower_bound = array_of_quartiles.at(2);
  }
  for(int i = 0;i<encoder_data_copy.size();++i){
    if(encoder_data_copy.at(i) > upper_bound || encoder_data_copy.at(i) < lower_bound || are_encoders_reliable_.at(i) == false){
      encoder_data_copy.at(i) = median;
      num_outliers_per_encoder_.at(i) = num_outliers_per_encoder_.at(i) + 1;
    }
    else{
      num_outliers_per_encoder_.at(i) = 0;
    }
  }
  return encoder_data_copy;
}
*/

core::EncoderData EncodersPreprocessor::detectOutliers(const core::EncoderData encoder_data){
  const uint8_t num_reliable_encoders = std::accumulate(are_encoders_reliable_.begin(),are_encoders_reliable_.end(),0);
  Quartile quartile;
  core::EncoderData encoder_data_copy;
  std::copy(encoder_data.begin(),encoder_data.end(),encoder_data_copy.begin());
  if(num_reliable_encoders == 3){
    std::array<uint32_t,3> encoder_data_copy_for_finding_quartiles;
    int counter = 0;
    for(int i = 0;i<encoder_data.size();++i){
      if(are_encoders_reliable_.at(i) == true){
        encoder_data_copy_for_finding_quartiles.at(counter) = encoder_data.at(i);
        counter = counter + 1;
      }
    }
    std::sort(encoder_data_copy_for_finding_quartiles.begin(),encoder_data_copy_for_finding_quartiles.end());
    quartile = getQuartiles(encoder_data_copy_for_finding_quartiles);// ask how to pass the value
  }
  else{
    core::EncoderData encoder_data_copy_for_finding_quartiles;
    std::copy(encoder_data.begin(),encoder_data.end(),encoder_data_copy_for_finding_quartiles.begin());
    std::sort(encoder_data_copy_for_finding_quartiles.begin(),encoder_data_copy_for_finding_quartiles.end());
    quartile = getQuartiles(encoder_data_copy_for_finding_quartiles); // ask how to pass the value
  }
  for(int i = 0;i<encoder_data_copy.size();++i){
    if(encoder_data_copy.at(i) > quartile.upperBound || encoder_data_copy.at(i) < quartile.lowerBound || are_encoders_reliable_.at(i) == false){
      encoder_data_copy.at(i) = quartile.median;
      num_outliers_per_encoder_.at(i) = num_outliers_per_encoder_.at(i) + 1;
    }
    else{
      num_outliers_per_encoder_.at(i) = 0;
    }
  }
  return encoder_data_copy;
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
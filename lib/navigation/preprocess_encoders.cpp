#include "preprocess_encoders.hpp"

#include <cmath>

#include <algorithm>
#include <array>
#include <numeric>

#include "core/types.hpp"

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

template<std::size_t N>
Quartile EncodersPreprocessor::getQuartiles(const std::array<std::uint32_t, N> &encoder_data)
{
  const std::uint32_t q1_high = static_cast<int>(std::ceil((encoder_data.size() + 1) / 4.0));
  const std::uint32_t q1_low  = static_cast<int>(std::floor((encoder_data.size() + 1) / 4.0));
  const std::uint32_t q3_high = static_cast<int>(std::ceil((3 * (encoder_data.size() + 1)) / 4.0));
  const std::uint32_t q3_low  = static_cast<int>(std::ceil((3 * (encoder_data.size() + 1)) / 4.0));
  const core::Float q1        = encoder_data.at(q1_low - 1)
                         + (((encoder_data.size() + 1) / 4.0) - q1_low)
                             * (encoder_data.at(q1_high - 1) - encoder_data.at(q1_low - 1));
  const core::Float q3 = encoder_data.at(q3_high - 1)
                         + (((3 * (encoder_data.size() + 1)) / 4.0) - q3_low)
                             * (encoder_data.at(q3_high - 1) - encoder_data.at(q3_low - 1));
  core::Float median;
  if (encoder_data.size() % 2 == 0) {
    median = (encoder_data.at((encoder_data.size() / 2) - 1)
              + encoder_data.at(((encoder_data.size() / 2) + 1)) - 1)
             / 2.0;
  } else {
    median = encoder_data.at(((encoder_data.size() + 1) / 2) - 1);
  }
  struct Quartile quartile;
  quartile.q1     = q1;
  quartile.median = median;
  quartile.q3     = q3;

  return quartile;  // return an output of the sequence median,upper_bound,lower_bound
}


core::EncoderData EncodersPreprocessor::detectOutliers(const core::EncoderData encoder_data)
{
  Quartile quartiles;
  core::EncoderData encoder_data_copy;
  std::copy(encoder_data.begin(), encoder_data.end(), encoder_data_copy.begin());
  if (num_reliable_encoders_ == core::kNumEncoders - 1) {
    std::array<uint32_t, core::kNumEncoders - 1> reliable_sensors_data;
    std::size_t counter = 0;
    for (std::size_t i = 0; i < encoder_data.size(); ++i) {
      if (are_encoders_reliable_.at(i) == true) {
        reliable_sensors_data.at(counter) = encoder_data.at(i);
        counter                                             = counter + 1;
      }
    }
    std::sort(reliable_sensors_data.begin(),
              reliable_sensors_data.end());
    quartiles = getQuartiles(reliable_sensors_data);  // ask how to pass the value
  } else {
    core::EncoderData reliable_sensors_data;
    std::copy(
      encoder_data.begin(), encoder_data.end(), reliable_sensors_data.begin());
    std::sort(reliable_sensors_data.begin(),
              reliable_sensors_data.end());
    quartiles = getQuartiles(reliable_sensors_data);  // ask how to pass the value
  }
  const core::Float iqr         = quartiles.q3 - quartiles.q1;
  const core::Float upper_bound = quartiles.median + 1.5 * iqr;
  const core::Float lower_bound = quartiles.median - 1.5 * iqr;

  for (std::size_t i = 0; i < encoder_data_copy.size(); ++i) {
    if (encoder_data_copy.at(i) > upper_bound || encoder_data_copy.at(i) < lower_bound
        || are_encoders_reliable_.at(i) == false) {
      encoder_data_copy.at(i)         = quartiles.median;
      num_outliers_per_encoder_.at(i) = num_outliers_per_encoder_.at(i) + 1;
    } else {
      num_outliers_per_encoder_.at(i) = 0;
    }
  }
  return encoder_data_copy;
}



void EncodersPreprocessor::checkReliable(const core::EncoderData &num_outliers_per_encoder_)
{
  for (int i = 0; i < core::kNumEncoders; i++) {
    if (num_outliers_per_encoder_.at(i) > 10) {  // for now assuming n to be 10
      are_encoders_reliable_.at(i) = false;     // the encoder is now unrealiable
    }
  }
}

}  // namespace hyped::navigation
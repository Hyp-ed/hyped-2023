#include "preprocess_encoders.hpp"

#include <cmath>

#include <algorithm>
#include <numeric>
#include <optional>

#include "core/types.hpp"

namespace hyped::navigation {

//EncodersPreprocessor::EncodersPreprocessor()
//{
  // TODOLater: implement
//}
EncodersPreprocessor::EncodersPreprocessor(core::ILogger &logger)
    : logger_(logger),
      encoder_outliers({0, 0, 0, 0}),
      encoders_reliable({true, true, true, true}),
      num_reliable_encoders_(core::kNumImus)
      {
      }

std::optional<core::EncoderData> EncodersPreprocessor::processData(const core::EncoderData encoder_data)
{
  /*
  TODOLater: implement
  basic plan:
  - call detectOutliers. Return of that function is return of this function
  - call checkRelaible

  */
  SensorChecks sensors  = checkReliable();
  if(sensors == SensorChecks::kUnacceptable){
    return std::nullopt;
  }
 return detectOutliers(encoder_data);
}

template<std::size_t N>
core::Float EncodersPreprocessor::getQuartile(const std::array<std::uint32_t, N> &reliable_data, const core::Float quartile_percent){

  const core::Float quartile_index = (num_reliable_encoders_ - 1)*quartile_percent;
  const std::uint8_t quartile_high = static_cast<int>(std::ceil(quartile_index));
  const std::uint8_t quartile_low = static_cast<int>(std::floor(quartile_index));
  const core::Float quartile = (reliable_data.at(quartile_high) + reliable_data.at(quartile_low))/2.0;
  return quartile;
}

template<std::size_t N>
Quartile EncodersPreprocessor::getQuartiles(std::array<std::uint32_t, N> &reliable_data){
  std::sort(reliable_data.begin(), reliable_data.end());
  Quartile quartile;
  quartile.q1 = getQuartile(reliable_data,0.25);
  quartile.median = getQuartile(reliable_data, 0.5);
  quartile.q3 = getQuartile(reliable_data, 0.75);
  return quartile;
}


core::EncoderData EncodersPreprocessor::detectOutliers(const core::EncoderData encoder_data)
{
  core::EncoderData encoder_data_copy;
  std::copy(encoder_data.begin(), encoder_data.end(), encoder_data_copy.begin());
  Quartile quartiles;
  if (num_reliable_encoders_ == core::kNumEncoders - 1) {
    std::array<uint32_t, core::kNumEncoders - 1> reliable_data;
    std::size_t counter = 0;
    for (std::size_t i = 0; i < encoder_data.size(); ++i) {
      if (encoders_reliable.at(i)) {
        reliable_data.at(counter) = encoder_data.at(i);
        ++counter;
      }
    }
    quartiles = getQuartiles(reliable_data);
  } else {
    core::EncoderData reliable_data;
    std::copy(
      encoder_data.begin(), encoder_data.end(), reliable_data.begin());
    quartiles = getQuartiles(reliable_data); 
  }
  const core::Float iqr         = quartiles.q3 - quartiles.q1;
  const core::Float upper_bound = quartiles.median + 1.5 * iqr;
  const core::Float lower_bound = quartiles.median - 1.5 * iqr;

  for (std::size_t i = 0; i < encoder_data_copy.size(); ++i) {
    if (encoder_data_copy.at(i) > upper_bound || encoder_data_copy.at(i) < lower_bound || encoders_reliable.at(i) == false) {
      encoder_data_copy.at(i)         = quartiles.median;

      if(encoders_reliable.at(i)){
        encoder_outliers.at(i) = encoder_outliers.at(i) + 1;
      }
    } else {
      encoder_outliers.at(i) = 0;
    }
  }

  return encoder_data_copy;
}



SensorChecks EncodersPreprocessor::checkReliable()
{
  for (int i = 0; i < core::kNumEncoders; ++i) {
    if (encoder_outliers.at(i) > max_consecutives) {  // for now assuming n to be 10
      encoders_reliable.at(i) = false;     // the encoder is now unrealiable
      --num_reliable_encoders_;
    }
  }
  if(num_reliable_encoders_ < core::kNumEncoders - 1){
    logger_.log(core::LogLevel::kFatal, "Sensors have become unreliable");
    return SensorChecks::kUnacceptable;
  }
  return SensorChecks::kAcceptable;
}

}  // namespace hyped::navigation
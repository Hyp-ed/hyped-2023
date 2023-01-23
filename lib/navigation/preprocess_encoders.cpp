#include "preprocess_encoders.hpp"

namespace hyped::navigation {

EncodersPreprocessor::EncodersPreprocessor(core::ILogger &logger)
    : logger_(logger),
      encoder_outliers_({0, 0, 0, 0}),
      encoders_reliable_({true, true, true, true}),
      num_reliable_encoders_(core::kNumImus)
      {
      }

std::optional<core::EncoderData> EncodersPreprocessor::processData(const core::EncoderData raw_encoder_data)
{
  /*
  TODOLater: implement
  basic plan:
  - call detectOutliers. Return of that function is return of this function
  - call checkRelaible

  */
  const auto encoder_data = detectOutliers(raw_encoder_data);
  SensorChecks sensors  = checkReliable();
  if(!encoder_data.has_value() || sensors == SensorChecks::kUnacceptable){
    return std::nullopt;
  } else{
    return encoder_data;
  }
}


std::optional<core::EncoderData> EncodersPreprocessor::detectOutliers(const core::EncoderData encoder_data)
{
  core::EncoderData encoder_data_copy;
  std::copy(encoder_data.begin(), encoder_data.end(), encoder_data_copy.begin());
  Quartile quartiles;
  core::Float interquartile_range; 
  core::Float upper_bound;
  core::Float lower_bound;
  //Case 1: All sensors are functioning properly
  if(num_reliable_encoders_ == core::kNumEncoders) {
    core::EncoderData reliable_data;
    std::copy(
      encoder_data.begin(), encoder_data.end(), reliable_data.begin());
    quartiles = getQuartiles(reliable_data); 
    interquartile_range        = quartiles.q3 - quartiles.q1;
    upper_bound = quartiles.median + 1.5 * interquartile_range ;
    lower_bound = quartiles.median - 1.5 * interquartile_range ;

  }
  //Case 2: One of the sensors has become faulty
  else if (num_reliable_encoders_ == core::kNumEncoders - 1) {
    std::array<uint32_t, core::kNumEncoders - 1> reliable_data;
    std::size_t counter = 0;
    for (std::size_t i = 0; i < encoder_data.size(); ++i) {
      if (encoders_reliable_.at(i)) {
        reliable_data.at(counter) = encoder_data.at(i);
        ++counter;
      }
    }
    quartiles = getQuartiles(reliable_data);
    interquartile_range        = quartiles.q3 - quartiles.q1;
    upper_bound = quartiles.median + 1.2 * interquartile_range ;
    lower_bound = quartiles.median - 1.2 * interquartile_range ;
  }
  //Case 3: More than one sensors have become unreliable, stopping everything and entering into fail State.
  else{
    logger_.log(core::LogLevel::kFatal, "Number of unreliable encoder sensors have exceeded the threshold");
    return std::nullopt;
  }

  for (std::size_t i = 0; i < encoder_data_copy.size(); ++i) {
    //replacing the ouliers or data from faulty sensors with the median of the dataset.
    if (encoder_data_copy.at(i) > upper_bound || encoder_data_copy.at(i) < lower_bound || encoders_reliable_.at(i) == false) {
      encoder_data_copy.at(i)         = quartiles.median;

      if(encoders_reliable_.at(i)){
        ++encoder_outliers_.at(i);
      }
    } else {
      encoder_outliers_.at(i) = 0;
    }
  }

  return encoder_data_copy;
}



SensorChecks EncodersPreprocessor::checkReliable()
{
  for (std::size_t i = 0; i < core::kNumEncoders; ++i) {
    // changes reliable sensor to false if max consecutive outliers are reached
    if (encoder_outliers_.at(i) > max_consecutive_outliers_ && encoders_reliable_.at(i) == true) {  // for now assuming n to be 10
      encoders_reliable_.at(i) = false;     // the encoder is now unrealiable
      --num_reliable_encoders_;
    }
  }
  if(num_reliable_encoders_ < core::kNumEncoders - 1){
    logger_.log(core::LogLevel::kFatal, "Number of unreliable encoder sensors have exceeded the threshold");
    return SensorChecks::kUnacceptable;
  }
  return SensorChecks::kAcceptable;
}

}  // namespace hyped::navigation
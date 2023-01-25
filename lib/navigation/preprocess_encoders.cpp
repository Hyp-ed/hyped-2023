#include "preprocess_encoders.hpp"

#include "core/types.hpp"

namespace hyped::navigation {

EncodersPreprocessor::EncodersPreprocessor(core::ILogger &logger)
    : logger_(logger),
      encoder_outliers_({0, 0, 0, 0}),
      encoders_reliable_({true, true, true, true}),
      num_reliable_encoders_(core::kNumImus)
{
}

std::optional<core::EncoderData> EncodersPreprocessor::processData(
  const core::EncoderData raw_encoder_data)
{
  const auto encoder_data = detectOutliers(raw_encoder_data);
  SensorChecks sensors    = checkReliable();
  if (!encoder_data.has_value() || sensors == SensorChecks::kUnacceptable) {
    return std::nullopt;
  } else {
    return encoder_data;
  }
}

std::optional<EncodersPreprocessor::Statistics> EncodersPreprocessor::getStatistics(
  const core::EncoderData &encoder_data)
{
  if (num_reliable_encoders_ > core::kNumEncoders
      || num_reliable_encoders_ < core::kNumEncoders - 1) {
    logger_.log(core::LogLevel::kFatal,
                "Unsuitable number of reliable encoders (%d of %d)",
                num_reliable_encoders_,
                core::kNumEncoders);
    return std::nullopt;
  }
  if (num_reliable_encoders_ == core::kNumEncoders) {
    std::array<std::uint32_t, core::kNumEncoders> reliable_data;
    std::copy(encoder_data.begin(), encoder_data.end(), reliable_data.begin());
    const Quartile quartiles              = getQuartiles(reliable_data);
    const core::Float interquartile_range = quartiles.q3 - quartiles.q1;
    return {
      {.median      = quartiles.median,
       .upper_bound = quartiles.median + static_cast<core::Float>(1.5) * interquartile_range,
       .lower_bound = quartiles.median - static_cast<core::Float>(1.5) * interquartile_range}};
  } else {
    std::array<uint32_t, core::kNumEncoders - 1> reliable_data;
    std::size_t j = 0;
    for (std::size_t i = 0; i < encoder_data.size(); ++i) {
      if (encoders_reliable_.at(i)) {
        reliable_data.at(j) = encoder_data.at(i);
        ++j;
      }
    }
    const Quartile quartiles              = getQuartiles(reliable_data);
    const core::Float interquartile_range = quartiles.q3 - quartiles.q1;
    return {
      {.median      = quartiles.median,
       .upper_bound = quartiles.median + static_cast<core::Float>(1.2) * interquartile_range,
       .lower_bound = quartiles.median - static_cast<core::Float>(1.2) * interquartile_range}};
  }
}

std::optional<core::EncoderData> EncodersPreprocessor::detectOutliers(
  const core::EncoderData &encoder_data)
{
  const auto statistics = getStatistics(encoder_data);
  if (!statistics) {
    logger_.log(core::LogLevel::kFatal, "Failed to obtain statistics for measurement");
  }
  core::EncoderData encoder_data_copy;
  std::copy(encoder_data.begin(), encoder_data.end(), encoder_data_copy.begin());
  for (std::size_t i = 0; i < encoder_data_copy.size(); ++i) {
    // replacing the ouliers or data from faulty sensors with the median of the dataset.
    if (encoder_data_copy.at(i) > statistics->upper_bound
        || encoder_data_copy.at(i) < statistics->lower_bound) {
      encoder_data_copy.at(i) = statistics->median;
      ++encoder_outliers_.at(i);
    } else if (!encoders_reliable_.at(i)) {
      encoder_data_copy.at(i) = statistics->median;
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
    if (encoder_outliers_.at(i) > max_consecutive_outliers_
        && encoders_reliable_.at(i) == true) {  // for now assuming n to be 10
      encoders_reliable_.at(i) = false;         // the encoder is now unrealiable
      --num_reliable_encoders_;
    }
  }
  if (num_reliable_encoders_ < core::kNumEncoders - 1) {
    logger_.log(core::LogLevel::kFatal,
                "Number of unreliable encoder sensors have exceeded the threshold");
    return SensorChecks::kUnacceptable;
  }
  return SensorChecks::kAcceptable;
}

}  // namespace hyped::navigation

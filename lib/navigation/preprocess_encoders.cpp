#include "preprocess_encoders.hpp"

namespace hyped::navigation {

EncodersPreprocessor::EncodersPreprocessor(core::ILogger &logger)
    : logger_(logger),
      num_consecutive_outliers_per_encoder_({0, 0, 0, 0}),
      are_encoders_reliable_({true, true, true, true}),
      num_reliable_encoders_(core::kNumEncoders),
      max_consecutive_outliers_(10)
{
}

std::optional<core::EncoderData> EncodersPreprocessor::processData(
  const core::EncoderData &raw_encoder_data)
{
  const auto encoder_data = sanitise(raw_encoder_data);
  const auto sensors      = checkReliable();
  if (sensors == SensorChecks::kUnacceptable) { return std::nullopt; }
  return encoder_data;
}

std::optional<EncodersPreprocessor::Statistics> EncodersPreprocessor::getStatistics(
  const core::EncoderData &encoder_data) const
{
  if (num_reliable_encoders_ < core::kNumEncoders - 1) {
    logger_.log(core::LogLevel::kFatal,
                "Unsuitable number of reliable encoders (%d of %d)",
                num_reliable_encoders_,
                core::kNumEncoders);
    return std::nullopt;
  }
  if (num_reliable_encoders_ == core::kNumEncoders) {
    auto reliable_data                    = encoder_data;
    const Quartile quartiles              = getQuartiles(reliable_data);
    const core::Float interquartile_range = quartiles.third_quartile - quartiles.first_quartile;
    return {{.median      = quartiles.median,
             .upper_bound = quartiles.median + 1.5 * interquartile_range,
             .lower_bound = quartiles.median - 1.5 * interquartile_range}};
  }
  std::array<std::uint32_t, core::kNumEncoders - 1> reliable_data;
  std::size_t j = 0;
  for (std::size_t i = 0; i < encoder_data.size(); ++i) {
    if (are_encoders_reliable_.at(i)) {
      reliable_data.at(j) = encoder_data.at(i);
      ++j;
    }
  }
  const Quartile quartiles              = getQuartiles(reliable_data);
  const core::Float interquartile_range = quartiles.third_quartile - quartiles.first_quartile;
  return {{.median      = quartiles.median,
           .upper_bound = quartiles.median + 1.2 * interquartile_range,
           .lower_bound = quartiles.median - 1.2 * interquartile_range}};
}

std::optional<core::EncoderData> EncodersPreprocessor::sanitise(
  const core::EncoderData &encoder_data)
{
  const auto encoder_statistics_optional = getStatistics(encoder_data);
  if (!encoder_statistics_optional) {
    logger_.log(core::LogLevel::kFatal, "Failed to obtain statistics for measurement");
    return std::nullopt;
  }
  const auto encoder_statistics = *encoder_statistics_optional auto sanitised_data = encoder_data;
  for (std::size_t i = 0; i < sanitised_data.size(); ++i) {
    if (sanitised_data.at(i) > encoder_statistics.upper_bound
        || sanitised_data.at(i) < encoder_statistics.lower_bound || !are_encoders_reliable_.at(i)) {
      ++num_consecutive_outliers_per_encoder_.at(i);
      sanitised_data.at(i) = encoder_statistics.median;
    } else {
      num_consecutive_outliers_per_encoder_.at(i) = 0;
    }
  }
  if (checkReliable() == SensorChecks::kUnacceptable) { return std::nullopt; }
  for (std::size_t i = 0; i < are_encoders_reliable_.size(); ++i) {
    if (!are_encoders_reliable_.at(i)) { sanitised_data.at(i) = encoder_statistics.median; }
  }
  return sanitised_data;
}

SensorChecks EncodersPreprocessor::checkReliable()
{
  if (num_reliable_encoders_ > core::kNumEncoders) {
    logger_.log(core::LogLevel::kFatal,
                "Number of reliable encoders is greater than number of encoders")
  }
  for (std::size_t i = 0; i < core::kNumEncoders; ++i) {
    // changes reliable sensor to false if max consecutive outliers are reached
    if (num_consecutive_outliers_per_encoder_.at(i) > max_consecutive_outliers_
        && are_encoders_reliable_.at(i)) {
      are_encoders_reliable_.at(i) = false;  // the encoder is now unrealiable
      logger_.log(core::LogLevel::kWarn,
                  "Sensor %d is now labelled as unreliable",
                  i)-- num_reliable_encoders_;
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

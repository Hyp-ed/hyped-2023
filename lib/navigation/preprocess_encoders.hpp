#pragma once
#include "consts.hpp"

#include <cmath>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

#include "core/logger.hpp"
#include "core/types.hpp"

namespace hyped::navigation {

class EncodersPreprocessor {
 public:
  EncodersPreprocessor(core::ILogger &logger);

  /**
   * @brief process encoder data to deal with outliers and unreliable sensors
   *
   * @param encoder_data
   * @return cleaned and reliable encoder data
   */
  std::optional<core::EncoderData> processData(const core::EncoderData &encoder_data);

 private:
  // TODO confirm or improve name
  struct Statistics {
    core::Float median;
    core::Float upper_bound;
    core::Float lower_bound;
  };

  // TODO Doc
  std::optional<Statistics> getStatistics(const core::EncoderData &encoder_data);

  /**
   * @brief tidies up the received data by detecting the outliers or data received from faulty
     sensors and replaces them with median value of the dataset
   *
   * @param encoder_data array containing the values received from various encoder sensors
   * @return consistent data with no outliers
   */
  std::optional<core::EncoderData> sanitise(const core::EncoderData &encoder_data);

  /**
   * @brief changes the value corresponding to the encoder sensor in the encoders_reliable_ array to
   * false if the number of consecutive outliers exceeds the threshold value
   *
   * @return SensorChecks::kUnacceptable if more than 1 sensors have become unreliable and
   * SensorChecks::kAcceptable otherwise
   */
  SensorChecks checkReliable();

  /**
   * @brief calculates a specific quantile value from sorted input array
   *
   * @param reliable_data sorted array of values
   * @param fraction in [0, 1] corresponding to the quantile (e.g. 0.25 for the 25th percentile)
   * @return the quantile that we require
   */
  template<std::size_t N>
  core::Float getSpecificQuantile(const std::array<std::uint32_t, N> &reliable_data,
                                  const core::Float fraction)
  {
    const core::Float theoretical_index = (num_reliable_encoders_ - 1) * fraction;
    const std::size_t low_index         = static_cast<std::size_t>(std::floor(theoretical_index));
    const std::size_t high_index        = static_cast<std::size_t>(std::ceil(theoretical_index));
    const auto low                      = static_cast<core::Float>(reliable_data.at(low_index));
    const auto high                     = static_cast<core::Float>(reliable_data.at(high_index));
    return (low + high) / 2.0;
  }

  /**
   * @brief cacluates the quartiles (q1,median,q3)
   *
   * @param array containing filtered values(i.e with no values from faulty sensors)
   * @return a quartile object which contains q1, q3 and the median value
   */
  template<std::size_t N>
  Quartile getQuartiles(std::array<std::uint32_t, N> &reliable_data)
  {
    std::sort(reliable_data.begin(), reliable_data.end());
    return {.q1     = getSpecificQuantile(reliable_data, 0.25),
            .median = getSpecificQuantile(reliable_data, 0.5),
            .q3     = getSpecificQuantile(reliable_data, 0.75)};
  }
  core::ILogger &logger_;
  std::array<uint16_t, core::kNumEncoders> num_consecutive_outliers_per_encoder_;
  std::array<bool, core::kNumEncoders> is_reliable_per_encoder_;
  std::uint8_t num_reliable_encoders_;
  const std::uint8_t max_consecutive_outliers_;
};

}  // namespace hyped::navigation

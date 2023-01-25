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
  std::optional<core::EncoderData> detectOutliers(const core::EncoderData &encoder_data);

  /**
   * @brief changes the value corresponding to the encoder sensor in the encoders_reliable_ array to
   * false if the number of consecutive outliers exceeds the threshold value
   *
   * @return SensorChecks::kUnacceptable if more than 1 sensors have become unreliable and
   * SensorChecks::kAcceptable otherwise
   */
  SensorChecks checkReliable();

  /**
   * @brief calculates a specific quantile value from input array
   *
   * @param reliable_data array containing filtered values(i.e with no values from faulty sensors)
   * @param quartile_percent value corresponding to the quantile that we want(0.25 for q1, 0.5 for
   * median and 0.75 for q3)
   * @return the quantile that we require
   */
  template<std::size_t N>
  core::Float getSpecificQuantile(const std::array<std::uint32_t, N> &reliable_data,
                                  const core::Float quartile_percent)
  {
    const core::Float quartile_index = (num_reliable_encoders_ - 1) * quartile_percent;
    const std::uint8_t quartile_high = static_cast<std::uint8_t>(std::ceil(quartile_index));
    const std::uint8_t quartile_low  = static_cast<std::uint8_t>(std::floor(quartile_index));
    const core::Float quartile
      = (reliable_data.at(quartile_high) + reliable_data.at(quartile_low)) / 2.0;
    return quartile;
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

  // number of reliable encoders, initialised as core::kNumEncoders
  std::uint8_t num_reliable_encoders_;

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumEncoders> encoder_outliers_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumEncoders> encoders_reliable_;

  // number of allowed consecutive outliers from a single encoder
  std::uint8_t max_consecutive_outliers_ = 10;

  // for logging fail state
  core::ILogger &logger_;
};

}  // namespace hyped::navigation

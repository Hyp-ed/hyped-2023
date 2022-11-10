#include "consts.hpp"

#include "core/logger.hpp"
#include "core/types.hpp"

namespace hyped::navigation {

class KeyencePreprocessor {
 public:
  KeyencePreprocessor(hyped::core::ILogger &log_);

  /**
   * @brief Checks that keyences have not disagreed twice in a row.
   *
   * @param keyence_data: inputs from Keyence Sensors
   * @return SensorChecks: enum indicating if Keyence Sensors have failed
   */
  SensorChecks checkKeyenceAgrees(const core::KeyenceData &keyence_data);

 private:
  hyped::core::ILogger &log_;
  // Flag for if the previous keyence measurements disagreed.
  bool has_keyence_disagreed_;
};

}  // namespace hyped::navigation
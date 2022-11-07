#include "consts.hpp"

#include <array>
#include <cstdint>

#include "core/types.hpp"
#include "core/logger.hpp"

namespace hyped::navigation {

class KeyencePreprocessor {
 public:
 /**
  * @brief Construct a new Keyence Preprocessor object
  */
  KeyencePreprocessor(hyped::core::ILogger &logger);

  SensorChecks checkKeyenceAgrees(const core::KeyenceData &keyence_data);

 private:
  hyped::core::ILogger &logger_;
  bool has_keyence_disagreed_;
};

}  // namespace hyped::navigation
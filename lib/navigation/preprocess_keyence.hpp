#include "consts.hpp"

#include <array>
#include <cstdint>

#include "core/types.hpp"

namespace hyped::navigation {

class PreprocessKeyence {
 public:
  PreprocessKeyence();

  SensorChecks checkKeyenceAgrees(const core::KeyenceData keyence_data);

 private:
  bool is_keyence_agreeing_;
};

}  // namespace hyped::navigation
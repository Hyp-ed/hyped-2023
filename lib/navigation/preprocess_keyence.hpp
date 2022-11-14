#pragma once

#include "consts.hpp"
#include "types.hpp"

#include <array>
#include <cstdint>

#include "core/types.hpp"

namespace hyped::navigation {

class KeyencePreprocessor {
 public:
  KeyencePreprocessor();

  KeyenceData processData(const core::RawKeyenceData &keyence_data);

 private:
  bool is_keyence_agreeing_;
};

}  // namespace hyped::navigation

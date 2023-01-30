#include "dummy_adc.hpp"

namespace hyped::utils {

std::optional<std::uint16_t> DummyAdc::readValue()
{
  return std::nullopt;
}

}  // namespace hyped::utils
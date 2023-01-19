#include <io/adc.hpp>

namespace hyped::utils {

class DummyAdc : public io::IAdc {
 public:
  DummyAdc()  = default;
  ~DummyAdc() = default;

  std::optional<std::uint16_t> readValue();
};

}  // namespace hyped::utils
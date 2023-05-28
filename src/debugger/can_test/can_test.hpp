#include <cstdint>

#include <core/logger.hpp>
#include <core/wall_clock.hpp>
#include <io/can.hpp>
#include <io/hardware_can.hpp>

namespace hyped::debug {

class CanTest {
 public:
  CanTest(core::Logger &logger);
  void run();

 private:
  void send();
  void receive();

  core::Logger &logger_;
};
}  // namespace hyped::debug
#include <gtest/gtest.h>

#include <core/logger.hpp>
#include <sensors/mux.hpp>
#include <utils/dummy_i2c.hpp>
#include <utils/dummy_i2c_sensor.hpp>
#include <utils/dummy_logger.hpp>

namespace hyped::test {

TEST(Mux, construction)
{
  static constexpr std::uint8_t kSize = 8;
  std::array<std::unique_ptr<sensors::II2cMuxSensor<std::uint8_t>>, kSize> sensors;
  for (auto &sensor : sensors) {
    sensor = std::make_unique<utils::DummyI2cSensor>();
  }
  utils::DummyI2c i2c;
  utils::DummyLogger logger;
  sensors::Mux<std::uint8_t, kSize> mux(logger, i2c, 0, sensors);
}

}  // namespace hyped::test

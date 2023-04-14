#include <memory>

#include <gtest/gtest.h>

#include <core/logger.hpp>
#include <sensors/i2c_mux.hpp>
#include <utils/dummy_i2c.hpp>
#include <utils/dummy_i2c_sensor.hpp>
#include <utils/dummy_logger.hpp>

namespace hyped::test {

TEST(I2cMux, construction)
{
  static constexpr std::uint8_t kSize = 8;
  std::array<std::unique_ptr<sensors::IMuxSensor<std::uint8_t>>, kSize> sensors;
  for (auto &sensor : sensors) {
    sensor = std::make_unique<utils::DummyI2cSensor>();
  }
  const auto i2c = std::make_shared<utils::DummyI2c>();
  utils::DummyLogger logger;
  const auto maybe_i2c_mux = sensors::I2cMux<std::uint8_t, kSize>::create(logger, i2c, 0, sensors);
  ASSERT_TRUE(maybe_i2c_mux);
  const auto mux = *maybe_i2c_mux;
}

}  // namespace hyped::test

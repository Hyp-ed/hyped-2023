#include <memory>

#include <gtest/gtest.h>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <sensors/analogue_mux.hpp>
#include <utils/dummy_adc_mux_sensor.hpp>
#include <utils/dummy_gpio.hpp>
#include <utils/dummy_logger.hpp>

namespace hyped::test {

TEST(AnalogueMux, construction)
{
  static constexpr std::uint8_t kSize = 16;
  std::array<std::unique_ptr<sensors::IMuxSensor<core::Float>>, kSize> sensors;
  for (auto &sensor : sensors) {
    sensor = std::make_unique<utils::DummyAdcMuxSensor>();
  }
  utils::DummyGpio dummy_gpio(
    [](const std::uint8_t) { return std::nullopt; },
    [](const std::uint8_t, const core::DigitalSignal) { return hyped::core::Result::kSuccess; });
  std::array<std::shared_ptr<io::IGpioWriter>, sensors::kNumSelectorPins> selector_pin_writers;
  for (auto &selector_pin_writer : selector_pin_writers) {
    selector_pin_writer = *dummy_gpio.getWriter(0);
  }
  const std::shared_ptr<io::IGpioWriter> disable_pin_writer = *dummy_gpio.getWriter(0);
  utils::DummyLogger logger;
  const auto maybe_analogue_mux = sensors::AnalogueMux<core::Float, kSize>::create(
    logger, selector_pin_writers, disable_pin_writer, sensors);
  ASSERT_TRUE(maybe_analogue_mux);
  const auto mux = *maybe_analogue_mux;
}

}  // namespace hyped::test
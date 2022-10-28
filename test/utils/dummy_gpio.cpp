#include <iostream>

#include <gtest/gtest.h>

#include <utils/dummy_gpio.hpp>

namespace hyped::test {

TEST(DummyGpio, construct)
{
  utils::DummyGpio dummy_gpio(
    [](const std::uint8_t) { return std::nullopt; },
    [](const std::uint8_t, const core::DigitalSignal) { return io::GpioWriteResult::kFailure; });
}

void testRead(utils::DummyGpio &dummy_gpio,
              const std::uint8_t pin,
              const std::string expected_output)
{
  testing::internal::CaptureStdout();
  auto dummy_gpio_reader_opt = dummy_gpio.getReader(pin);
  ASSERT_TRUE(dummy_gpio_reader_opt);
  auto dummy_gpio_reader = *dummy_gpio_reader_opt;
  ASSERT_TRUE(dummy_gpio_reader);
  const auto value = dummy_gpio_reader->read();
  ASSERT_EQ(value, core::DigitalSignal::kHigh);
  ASSERT_EQ(testing::internal::GetCapturedStdout(), expected_output);
}

void testWrite(utils::DummyGpio &dummy_gpio,
               const std::uint8_t pin,
               const core::DigitalSignal state,
               const std::string expected_output)
{
  testing::internal::CaptureStdout();
  auto dummy_gpio_writer_opt = dummy_gpio.getWriter(pin);
  ASSERT_TRUE(dummy_gpio_writer_opt);
  auto dummy_gpio_writer = *dummy_gpio_writer_opt;
  ASSERT_TRUE(dummy_gpio_writer);
  const auto result = dummy_gpio_writer->write(state);
  ASSERT_EQ(result, io::GpioWriteResult::kSuccess);
  ASSERT_EQ(testing::internal::GetCapturedStdout(), expected_output);
}

TEST(DummyGpio, printToStderr)
{
  // dummy GPIO that prints to stdout whenever the interface is accessed
  utils::DummyGpio dummy_gpio(
    [](const std::uint8_t pin) {
      std::cout << "read " << static_cast<int>(pin) << std::endl;
      return core::DigitalSignal::kHigh;
    },
    [](const std::uint8_t pin, const core::DigitalSignal state) {
      std::cout << "wrote " << static_cast<int>(pin) << " ";
      switch (state) {
        case core::DigitalSignal::kLow:
          std::cout << "low";
          break;
        case core::DigitalSignal::kHigh:
          std::cout << "high";
      }
      std::cout << std::endl;
      return io::GpioWriteResult::kSuccess;
    });
  testRead(dummy_gpio, 4, "read 4\n");
  testRead(dummy_gpio, 42, "read 42\n");
  testRead(dummy_gpio, 255, "read 255\n");
  testRead(dummy_gpio, 0, "read 0\n");
  testWrite(dummy_gpio, 4, core::DigitalSignal::kHigh, "wrote 4 high\n");
  testWrite(dummy_gpio, 42, core::DigitalSignal::kLow, "wrote 42 low\n");
  testWrite(dummy_gpio, 255, core::DigitalSignal::kHigh, "wrote 255 high\n");
  testWrite(dummy_gpio, 0, core::DigitalSignal::kLow, "wrote 0 low\n");
}

}  // namespace hyped::test

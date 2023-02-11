#include <iostream>

#include <gtest/gtest.h>

#include <utils/dummy_i2c.hpp>

namespace hyped::test {

TEST(DummyI2c, construction)
{
  utils::DummyI2c dummy_i2c(
    [](const std::uint8_t, const std::uint8_t) { return std::nullopt; },
    [](const std::uint8_t, const std::uint8_t) { return core::Result::kSuccess; },
    [](const std::uint8_t, const std::uint8_t, const std::uint8_t) {
      return core::Result::kSuccess;
    });
}

}  // namespace hyped::test

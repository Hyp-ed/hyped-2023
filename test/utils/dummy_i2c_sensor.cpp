#include <iostream>

#include <gtest/gtest.h>

#include <utils/manual_time.hpp>
#include <utils/dummy_i2c_sensor.hpp>

namespace hyped::test {

TEST(DummyI2cSensor, construction)
{
  utils::ManualTime manual_time;
  utils::DummyI2cSensor dummy_i2c_sensor(manual_time);
}

}  // namespace hyped::test

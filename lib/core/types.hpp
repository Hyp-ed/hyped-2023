#pragma once
#include <array>
#include <cstdint>
namespace hyped::core {

enum class DigitalSignal { kLow = 0, kHigh };

using Float = float;

struct CanFrame {
  uint32_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  uint8_t can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  uint8_t __pad;   /* padding */
  uint8_t __res0;  /* reserved / padding */
  uint8_t __res1;  /* reserved / padding */
  std::array<uint8_t, 8> data;
};

struct ImuData{
  float imu0;
  float imu1;
  float imu2;
  float imu3;
};

struct WheelEncoderData{
  uint32_t encoder0;
  uint32_t encoder1;
  uint32_t encoder2;
  uint32_t encoder3;
};

struct KeyenceData{
  uint32_t keyence0;
  uint32_t keyence1;
};

}  // namespace hyped::core

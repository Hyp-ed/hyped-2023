#pragma once
#include <array>
#include <cstdint>
namespace hyped::core {

enum class DigitalSignal { kLow = 0, kHigh };

using Float = float;

#if LINUX
#include <linux/can.h>
struct CanFrame = can_frame;
#else
struct CanFrame {
  uint32_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  uint8_t can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  uint8_t __pad;   /* padding */
  uint8_t __res0;  /* reserved / padding */
  uint8_t __res1;  /* reserved / padding */
  std::array<uint8_t, 8> data;
};
#endif

}  // namespace hyped::core

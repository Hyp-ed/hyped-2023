#pragma once
#include <stdint.h>

namespace hyped::core {

enum class LowOrHigh { kLow = 0, kHigh };

struct CanFrame
{
    uint32_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    uint8_t can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    uint8_t __pad;   /* padding */
    uint8_t __res0;  /* reserved / padding */
    uint8_t __res1;  /* reserved / padding */
    uint8_t data[8];
};
}  // namespace hyped::core

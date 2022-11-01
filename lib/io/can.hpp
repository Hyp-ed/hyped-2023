#include <string>

#include <core/logger.hpp>
#include <core/types.hpp>

#ifndef LINUX
#include <linux/can.h>
#endif
namespace hyped::io {

#ifndef LINUX
using CanFrame = can_frame;
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

enum class CanResult { kFailure, kSuccess };

class ICan {
  virtual CanResult initialise(const std::string &can_network_interface);
  virtual CanResult send(const CanFrame &message);
  virtual std::optional<CanFrame> receive();
};

class Can : ICan {
 public:
  Can(core::ILogger &logger);
  CanResult initialise(const std::string &can_network_interface);
  CanResult send(const CanFrame &message);
  std::optional<CanFrame> receive();

 private:
  int socket_;
  hyped::core::ILogger &logger_;
};
}  // namespace hyped::io

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

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

class ICanProcessor {
 public:
  virtual void processMessage(const io::CanFrame &frame) = 0;
};

class ICan {
 public:
  virtual CanResult initialise(const std::string &can_network_interface)   = 0;
  virtual CanResult send(const CanFrame &message)                          = 0;
  virtual std::optional<CanFrame> receive()                                = 0;
  virtual void listen()                                                    = 0;
  virtual void addCanProcessor(uint16_t ID, ICanProcessor &processor)      = 0;
};

class Can : ICan {
 public:
  Can(core::ILogger &logger);
  CanResult initialise(const std::string &can_network_interface);
  CanResult send(const CanFrame &message);
  std::optional<CanFrame> receive();
  void listen();
  void addCanProcessor(const uint16_t id, std::shared_ptr<ICanProcessor> processor);

 private:
  int socket_;
  hyped::core::ILogger &logger_;
  std::unordered_map<uint32_t, std::vector<std::shared_ptr<ICanProcessor>>> processors_;
};

}  // namespace hyped::io

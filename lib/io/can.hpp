#pragma once
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <core/logger.hpp>
#include <core/types.hpp>

#ifdef __linux__
#include <linux/can.h>
#endif
namespace hyped::io {

#ifdef __linux__
using CanFrame = can_frame;
#else
// structs and values as defined in <linux/can.h>
struct CanFrame {
  std::uint32_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  std::uint8_t can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  std::uint8_t __pad;   /* padding */
  std::uint8_t __res0;  /* reserved / padding */
  std::uint8_t __res1;  /* reserved / padding */
  std::array<std::uint8_t, 8> data;
};
struct sockaddr_can {
  std::uint8_t can_family;
  std::uint8_t can_ifindex;
  union {
    /* transport protocol class address info (e.g. ISOTP) */
    struct {
      std::uint32_t rx_id, tx_id;
    } tp;
    /* reserved for future CAN protocols address information */
  } can_addr;
};
#define PF_CAN 29
#define AF_CAN 29
#define CAN_RAW 1
#endif

class ICanProcessor {
 public:
  virtual void processMessage(const io::CanFrame &frame) = 0;
};

class ICan {
 public:
  virtual core::Result initialise(const std::string &can_network_interface) = 0;
  virtual core::Result send(const CanFrame &message)                        = 0;
  virtual std::optional<CanFrame> receive()                                 = 0;
  virtual core::Result listen()                                             = 0;
  virtual void addCanProcessor(const std::uint16_t id, ICanProcessor &processor) = 0;
};

class Can : public ICan {
 public:
  Can(core::ILogger &logger);
  core::Result initialise(const std::string &can_network_interface);
  core::Result send(const CanFrame &message);
  std::optional<CanFrame> receive();
  core::Result listen();
  void addCanProcessor(const std::uint16_t id, std::shared_ptr<ICanProcessor> processor);

 private:
  int socket_;
  core::ILogger &logger_;
  std::unordered_map<std::uint32_t, std::vector<std::shared_ptr<ICanProcessor>>> processors_;
};

}  // namespace hyped::io

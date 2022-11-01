#include "can.hpp"

#include <stdio.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace hyped::io {
Can::Can(core::ILogger &logger) : logger_(logger)
{
}

CanResult Can::initialise(const std::string &can_network_interface)
{
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Unable to open CAN socket");
    return io::CanResult::kFailure;
  }
  sockaddr_can socket_address;
  socket_address.can_family  = AF_CAN;
  socket_address.can_ifindex = if_nametoindex(can_network_interface.c_str());
  if (socket_address.can_ifindex == 0) {
    logger_.log(core::LogLevel::kFatal, "Unable to find CAN1 network interface");
    close(socket_);
    socket_ = -1;
    return io::CanResult::kFailure;
  }
  const int bind_status
    = bind(socket_, reinterpret_cast<sockaddr *>(&socket_address), sizeof(socket_address));
  if (bind_status < 0) {
    logger_.log(core::LogLevel::kFatal, "Unable to bind CAN socket");
    close(socket_);
    socket_ = -1;
    return io::CanResult::kFailure;
  }
  logger_.log(core::LogLevel::kInfo, "CAN socket successfully created");
  return io::CanResult::kSuccess;
}

CanResult Can::send(const CanFrame &message)
{
  if (socket_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Trying to send CAN message but no CAN socket found");
    return io::CanResult::kFailure;
  }
  const int num_bytes_written = write(socket_, &message, sizeof(can_frame));
  if (num_bytes_written != sizeof(can_frame)) {
    logger_.log(core::LogLevel::kFatal, "Failed to send CAN message");
    return io::CanResult::kFailure;
  }
  // TODO make logger more elegant
  logger_.log(core::LogLevel::kDebug,
              "CAN message sent, ID:%i, DATA: %i %i %i %i %i %i %i %i",
              static_cast<int>(message.can_id),
              static_cast<int>(message.data[0]),
              static_cast<int>(message.data[1]),
              static_cast<int>(message.data[2]),
              static_cast<int>(message.data[3]),
              static_cast<int>(message.data[4]),
              static_cast<int>(message.data[5]),
              static_cast<int>(message.data[6]),
              static_cast<int>(message.data[7]));
  return io::CanResult::kSuccess;
}

std::optional<CanFrame> Can::receive()
{
  CanFrame received_message;
  const int num_bytes_read = read(socket_, &received_message, sizeof(can_frame));
  if (num_bytes_read < sizeof(can_frame)) {
    logger_.log(core::LogLevel::kFatal, "Failed to receive CAN message");
    return std::nullopt;
  }
  return received_message;
}
}  // namespace hyped::io

#include "can.hpp"

#include <stdio.h>
#include <unistd.h>

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace hyped::io {
Can::Can(hyped::core::ILogger &logger) : logger_(logger)
{
}

CanResult Can::initialiseCanSocket(std::string can_network_interface)
{
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    logger_.log(hyped::core::LogLevel::kFatal, "Unable to open CAN socket");
    return hyped::io::CanResult::kFailure;
  }
  sockaddr_can socket_address;
  socket_address.can_family  = AF_CAN;
  socket_address.can_ifindex = if_nametoindex(can_network_interface.c_str());
  if (socket_address.can_ifindex == 0) {
    logger_.log(hyped::core::LogLevel::kFatal, "Unable to find CAN1 network interface");
    close(socket_);
    socket_ = -1;
    return hyped::io::CanResult::kFailure;
  }
  const int bind_status
    = bind(socket_, reinterpret_cast<sockaddr *>(&socket_address), sizeof(socket_address));
  if (bind_status < 0) {
    logger_.log(hyped::core::LogLevel::kFatal, "Unable to bind CAN socket");
    close(socket_);
    socket_ = -1;
    return hyped::io::CanResult::kFailure;
  }
  logger_.log(hyped::core::LogLevel::kInfo, "CAN socket successfully created");
  return hyped::io::CanResult::kSuccess;
}

CanResult Can::sendCanFrame(const core::CanFrame message)
{
  if (socket_ < 0) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Trying to send CAN message but no CAN socket found");
    return hyped::io::CanResult::kFailure;
  }
  const int num_bytes_written = write(socket_, &message, sizeof(can_frame));
  if (num_bytes_written != sizeof(can_frame)) {
    logger_.log(hyped::core::LogLevel::kFatal, "Failed to send CAN message");
  }
  logger_.log(hyped::core::LogLevel::kDebug,
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
  return hyped::io::CanResult::kSuccess;
}

std::optional<can_frame> Can::receiveCanFrame()
{
  can_frame received_message;
  const int num_bytes_read = read(socket_, &received_message, sizeof(can_frame));
  if (num_bytes_read < sizeof(can_frame)) {
    logger_.log(hyped::core::LogLevel::kFatal, "Failed to receive CAN message");
    return std::nullopt;
  }
  return received_message;
}
}  // namespace hyped::io

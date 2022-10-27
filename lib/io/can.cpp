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
  if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    logger_.log(hyped::core::LogLevel::kFatal, "Unable to open CAN socket");
    return;
  }

  sockaddr_can socket_address;
  socket_address.can_family  = AF_CAN;
  socket_address.can_ifindex = if_nametoindex("can1");

  if (socket_address.can_ifindex == 0) {
    logger_.log(hyped::core::LogLevel::kFatal, "Unable to find CAN1 network interface");
    close(socket_);
    socket_ = -1;
    return;
  }

  if (bind(socket_, (sockaddr *)&socket_address, sizeof(socket_address)) < 0) {
    logger_.log(hyped::core::LogLevel::kFatal, "Unable to bind CAN socket");
    close(socket_);
    socket_ = -1;
    return;
  }

  logger_.log(hyped::core::LogLevel::kInfo, "CAN socket successfully created");
}

int Can::sendCanFrame(can_frame message)
{
  if (socket_ < 0) {
    // log no can device found
    logger_.log(hyped::core::LogLevel::kFatal,
                "Trying to send CAN message but no CAN socket found");
    return;
  }

  if (write(socket_, &message, sizeof(can_frame)) != sizeof(can_frame)) {
    // log message not sent
    logger_.log(hyped::core::LogLevel::kFatal, "Failed to send CAN message");
  }

  logger_.log(hyped::core::LogLevel::kDebug,
              "CAN message sent, ID:%i, DATA: %i %i %i %i %i %i %i %i",
              (int)message.can_id,
              (int)message.data[0],
              (int)message.data[1],
              (int)message.data[2],
              (int)message.data[3],
              (int)message.data[4],
              (int)message.data[5],
              (int)message.data[6],
              (int)message.data[7]);
}

can_frame Can::receiveCanFrame()
{
  can_frame received_message;
  if (read(socket_, &received_message, sizeof(can_frame)) < sizeof(can_frame)) {
    logger_.log(hyped::core::LogLevel::kFatal, "Failed to receive CAN message");
  }
  return received_message;
}
}  // namespace hyped::io

#include "hardware_can.hpp"

#include <stdio.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace hyped::io {

Can::Can(const std::string &can_network_interface, core::ILogger &logger)
    : logger_(logger),
      processors_()
{
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Unable to open CAN socket");
    return core::Result::kFailure;
  }
  const std::uint8_t interface_index = if_nametoindex(can_network_interface.c_str());
  if (!interface_index) {
    logger_.log(core::LogLevel::kFatal, "Unable to find CAN1 network interface");
    close(socket_);
    socket_ = -1;
    return core::Result::kFailure;
  }
  const sockaddr_can socket_address = {AF_CAN, interface_index};
  const int bind_status
    = bind(socket_, reinterpret_cast<const sockaddr *>(&socket_address), sizeof(socket_address));
  if (bind_status < 0) {
    logger_.log(core::LogLevel::kFatal, "Unable to bind CAN socket");
    close(socket_);
    socket_ = -1;
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kInfo, "CAN socket successfully created");
  return core::Result::kSuccess;
}

core::Result Can::send(const io::CanFrame &data)
{
  if (socket_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Trying to send CAN data but no CAN socket found");
    return core::Result::kFailure;
  }
  const int num_bytes_written = write(socket_, &data, sizeof(CanFrame));
  if (num_bytes_written != sizeof(CanFrame)) {
    logger_.log(core::LogLevel::kFatal, "Failed to send CAN data");
    return core::Result::kFailure;
  }
  // TODOLater make logger_ more elegant
  logger_.log(core::LogLevel::kDebug,
              "CAN data sent, ID:%i, DATA: %i %i %i %i %i %i %i %i",
              static_cast<int>(data.can_id),
              static_cast<int>(data.data[0]),
              static_cast<int>(data.data[1]),
              static_cast<int>(data.data[2]),
              static_cast<int>(data.data[3]),
              static_cast<int>(data.data[4]),
              static_cast<int>(data.data[5]),
              static_cast<int>(data.data[6]),
              static_cast<int>(data.data[7]));
  return core::Result::kSuccess;
}

std::optional<CanFrame> Can::receive()
{
  CanFrame received_message;
  if (ioctl(socket_, FIONREAD) < sizeof(CanFrame)) {
    logger_.log(core::LogLevel::kDebug, "No CAN data in rx queue");
    return std::nullopt;
  }
  const int num_bytes_read = read(socket_, &received_message, sizeof(CanFrame));
  if (num_bytes_read < sizeof(CanFrame)) {
    logger_.log(core::LogLevel::kFatal, "Failed to receive CAN data");
    return std::nullopt;
  }
  // TODOLater make logger more elegant
  logger_.log(core::LogLevel::kDebug,
              "CAN message received, ID:%i, DATA: %i %i %i %i %i %i %i %i",
              static_cast<int>(received_message.can_id),
              static_cast<int>(received_message.data[0]),
              static_cast<int>(received_message.data[1]),
              static_cast<int>(received_message.data[2]),
              static_cast<int>(received_message.data[3]),
              static_cast<int>(received_message.data[4]),
              static_cast<int>(received_message.data[5]),
              static_cast<int>(received_message.data[6]),
              static_cast<int>(received_message.data[7]));

  return received_message;
}

core::Result Can::listen()
{
  if (ioctl(socket_, FIONREAD) < sizeof(CanFrame)) { return core::Result::kFailure; }
  const auto message = receive();
  if (!message) { return core::Result::kFailure; }
  const auto subscribed_processors = processors_.find(message->can_id);
  if (subscribed_processors == processors_.end()) {
    logger_.log(core::LogLevel::kFatal, "No CanProccessor associated with id %i", message->can_id);
    return core::Result::kFailure;
  }
  for (auto &processor : subscribed_processors->second) {
    processor->processMessage(*message);
  }
  return core::Result::kSuccess;
}

void Can::addCanProcessor(const std::uint16_t id, std::shared_ptr<ICanProcessor> processor)
{
  const auto id_and_processors = processors_.find(id);
  if (id_and_processors == processors_.end()) {
    auto new_processor_vector = std::vector<std::shared_ptr<ICanProcessor>>({processor});
    processors_.emplace(id, new_processor_vector);
  } else {
    id_and_processors->second.push_back(processor);
  }
  logger_.log(core::LogLevel::kInfo, "Added processor for id %i", id);
}

}  // namespace hyped::io

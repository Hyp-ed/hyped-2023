#include "can.hpp"

#include <stdio.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace hyped::io {

Can::Can(core::ILogger &log_) : log_(log_), processors_()
{
}

io::CanResult Can::initialise(const std::string &can_network_interface)
{
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    log_.log(core::LogLevel::kFatal, "Unable to open CAN socket");
    return io::CanResult::kFailure;
  }
  sockaddr_can socket_address;
  socket_address.can_family  = AF_CAN;
  socket_address.can_ifindex = if_nametoindex(can_network_interface.c_str());
  if (socket_address.can_ifindex == 0) {
    log_.log(core::LogLevel::kFatal, "Unable to find CAN1 network interface");
    close(socket_);
    socket_ = -1;
    return io::CanResult::kFailure;
  }
  const int bind_status
    = bind(socket_, reinterpret_cast<sockaddr *>(&socket_address), sizeof(socket_address));
  if (bind_status < 0) {
    log_.log(core::LogLevel::kFatal, "Unable to bind CAN socket");
    close(socket_);
    socket_ = -1;
    return io::CanResult::kFailure;
  }
  log_.log(core::LogLevel::kInfo, "CAN socket successfully created");
  return io::CanResult::kSuccess;
}

io::CanResult Can::send(const io::CanFrame &data)
{
  if (socket_ < 0) {
    log_.log(core::LogLevel::kFatal, "Trying to send CAN data but no CAN socket found");
    return io::CanResult::kFailure;
  }
  const int num_bytes_written = write(socket_, &data, sizeof(CanFrame));
  if (num_bytes_written != sizeof(CanFrame)) {
    log_.log(core::LogLevel::kFatal, "Failed to send CAN data");
    return io::CanResult::kFailure;
  }
  // TODOLater make log_ more elegant
  log_.log(core::LogLevel::kDebug,
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
  return io::CanResult::kSuccess;
}

std::optional<CanFrame> Can::receive()
{
  CanFrame received_message;
  if (ioctl(socket_, FIONREAD) < sizeof(CanFrame)) {
    log_.log(core::LogLevel::kDebug, "No can data in rx queue");
    return std::nullopt;
  }
  const int num_bytes_read = read(socket_, &received_message, sizeof(CanFrame));
  if (num_bytes_read < sizeof(CanFrame)) {
    log_.log(core::LogLevel::kFatal, "Failed to receive CAN data");
    return std::nullopt;
  }
  // TODOLater make log_ more elegant
  log_.log(core::LogLevel::kDebug,
           "CAN data received, ID:%i, DATA: %i %i %i %i %i %i %i %i",
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

CanResult Can::listen()
{
  if (ioctl(socket_, FIONREAD) < sizeof(CanFrame)) { return io::CanResult::kFailure; }
  const auto data = receive();
  if (!data) { return io::CanResult::kFailure; }
  CanFrame message = data.value();
  if (!processors_.contains(message.can_id)) {
    log_.log(core::LogLevel::kInfo, "No CanProccessor associated with id %i", message.can_id);
    return io::CanResult::kFailure;
  }
  std::vector<std::shared_ptr<ICanProcessor>> &subscribed_processors = processors_[message.can_id];
  for (auto &processor : subscribed_processors) {
    processor->processMessage(message);
  }
  return io::CanResult::kSuccess;
}

void Can::addCanProcessor(const uint16_t id, std::shared_ptr<ICanProcessor> processor)
{
  processors_[id].push_back(processor);
  log_.log(core::LogLevel::kInfo, "Added processor for id %i", id);
}

}  // namespace hyped::io

#include <imd.hpp>

namespace hyped::sensors {
std::optional<std::shared_ptr<Imd>> Imd::create(core::ILogger &logger,
                                                const std::shared_ptr<io::ICan> can)
{
  std::optional<std::shared_ptr<Imd>> optional_imd = std::make_shared<Imd>(logger, can);
  if (!optional_imd) {
    logger.log(core::LogLevel::kFatal, "Failed to create IMD");
    return std::nullopt;
  }
  std::shared_ptr<Imd> imd = *optional_imd;
  can->addProcessor(kReturnDataCanId, imd);
  return imd;
}

Imd::Imd(core::ILogger &logger, const std::shared_ptr<io::ICan> can) : logger_(logger), can_(can)
{
}

core::Result Imd::updateValues()
{
  io::CanFrame frame;
  frame.can_id = CAN_EFF_FLAG | kRequestDataCanId;
  // Always one byte for Request_mux
  frame.can_dlc = 1;
  frame.data[0] = kRequestIsolationResistances;
  // Fill remaining data bits with 0
  for (std::size_t i = 1; i < 8; ++i) {
    frame.data[i] = 0;
  }
  core::Result result = can_->send(frame);
  if (result == core::Result::kFailure) {
    logger_.log(core::LogLevel::kFatal, "Failed to send update request over CAN");
  }
  return result;
}

core::Result Imd::processMessage(const io::CanFrame &frame)
{
  // Isolation status is stored across lowest two bits of byte 1
  // rp and rn are each stored across two bytes
  isolation_status_    = frame.data[1] & 3;
  resistance_positive_ = (static_cast<std::uint16_t>(frame.data[2]) << 8) | frame.data[3];
  resistance_negative_ = (static_cast<std::uint16_t>(frame.data[5]) << 8) | frame.data[6];
  return core::Result::kSuccess;
}

std::uint16_t Imd::getResistancePositive()
{
  return resistance_positive_;
}

std::uint16_t Imd::getResistanceNegative()
{
  return resistance_negative_;
}

std::uint8_t Imd::getIsolationStatus()
{
  return isolation_status_;
}

}  // namespace hyped::sensors
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
  can.addProcessor(kReturnDataCanId, imd);
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
  frame.can_dlc       = 1;
  frame.data[0]       = kRequestIsolationResistances;
  frame.data[1]       = 0;
  frame.data[2]       = 0;
  frame.data[3]       = 0;
  frame.data[4]       = 0;
  frame.data[5]       = 0;
  frame.data[6]       = 0;
  frame.data[7]       = 0;
  core::Result result = can_.send(frame);
  if (result == core::Result::kFailure) {
    logger_.log(core::LogLevel::kFatal, "Failed to send update request over CAN");
  }
  return result;
}

core::Result Imd::processMessage(const io::CanFrame &frame)
{
  // Isolation status is stored across 2 bits, while rp and rn are each stored across 2 bytes
  iso_state_ = (frame.data[1] & 1) | ((frame.data[1] & 2) << 1);
  rp_        = ((std::uint16_t)frame.data[3] << 8) | frame.data[2];
  rn_        = ((std::uint16_t)frame.data[6] << 8) | frame.data[5];
  return core::Result::kSuccess;
}

std::uint16_t Imd::getRp()
{
  return rp_;
}

std::uint16_t Imd::getRn()
{
  return rn_;
}

std::uint8_t Imd::getIsolationState()
{
  return iso_state_;
}

}  // namespace hyped::sensors
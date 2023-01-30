#include "bms_processor.hpp"

namespace hyped::sensors {

BmsProcessor::BmsProcessor(core::ILogger &logger) : logger_(logger)
{
}

void BmsProcessor::processMessage(const io::CanFrame &frame)
{
  // TODO: get BMS data
}

}  // namespace hyped::sensors
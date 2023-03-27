#pragma once

#include <core/logger.hpp>
#include <io/can.hpp>

// Current plan: Make a processor based on ICanProcessor

namespace hyped::sensors {
class ImdProcessor : public io::ICanProcessor {
 public:
  ImdProcessor(core::ILogger &logger);
  ImdProcessor();

  void processMessage(const io::CanFrame &frame);

 private:
  const core::ILogger &logger_;
};
}  // namespace hyped::sensors
#pragma once

#include <core/logger.hpp>
#include <io/can.hpp>

namespace hyped::sensors {
class BmsProcessor : public io::ICanProcessor {
 public:
  BmsProcessor(core::ILogger &logger);
  ~BmsProcessor() = default;
  void processMessage(const io::CanFrame &frame) override;

 private:
  core::ILogger &logger_;
};
}  // namespace hyped::sensors
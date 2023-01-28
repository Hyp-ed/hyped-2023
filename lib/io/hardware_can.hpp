#pragma once
#include "can.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {

class HardwareCan : public ICan {
 public:
  static std::optional<std::shared_ptr<HardwareCan>> create(
    core::ILogger &logger, const std::string &can_network_interface);
  ~HardwareCan();
  core::Result send(const CanFrame &message);
  std::optional<CanFrame> receive();
  core::Result listen();
  void addProcessor(const std::uint16_t id, std::shared_ptr<ICanProcessor> processor);

 private:
  HardwareCan(core::ILogger &logger, const std::int16_t socket);
  int socket_;
  core::ILogger &logger_;
  std::unordered_map<std::uint32_t, std::vector<std::shared_ptr<ICanProcessor>>> processors_;
};

}  // namespace hyped::io

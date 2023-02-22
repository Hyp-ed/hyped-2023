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
  HardwareCan(core::ILogger &logger, const int socket);
  ~HardwareCan();
  core::Result send(const CanFrame &message);
  core::Result receive();
  void addProcessor(const std::uint16_t id, std::shared_ptr<ICanProcessor> processor);

 private:
  core::ILogger &logger_;
  const int socket_;
  std::unordered_map<std::uint32_t, std::vector<std::shared_ptr<ICanProcessor>>> processors_;
};

}  // namespace hyped::io

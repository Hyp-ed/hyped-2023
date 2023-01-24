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
class Can : public ICan {
 public:
  Can(core::ILogger &logger);
  core::Result initialise(const std::string &can_network_interface);
  core::Result send(const CanFrame &message);
  std::optional<CanFrame> receive();
  core::Result listen();
  void addCanProcessor(const std::uint16_t id, std::shared_ptr<ICanProcessor> processor);

 private:
  int socket_;
  core::ILogger &logger_;
  std::unordered_map<std::uint32_t, std::vector<std::shared_ptr<ICanProcessor>>> processors_;
};

}  // namespace hyped::io

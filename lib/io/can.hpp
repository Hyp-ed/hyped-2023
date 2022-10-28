#include <string>

#include <core/logger.hpp>
#include <core/types.hpp>

#if LINUX
#include <linux/can.h>
#endif

namespace hyped::io {

enum class CanResult { kFailure, kSuccess };
class Can {
 public:
  Can(hyped::core::ILogger &logger);
  CanResult initialise(const std::string &can_network_interface);
  CanResult sendCanFrame(const core::CanFrame &message);
  std::optional<can_frame> receiveCanFrame();

 private:
  int socket_;
  hyped::core::ILogger &logger_;
};
}  // namespace hyped::io

#include <string>

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {

enum CanResult { kFailure, kSuccess };
class Can {
 public:
  Can(hyped::core::ILogger &logger);
  CanResult initialiseCanSocket(std::string can_network_interface);
  CanResult sendCanFrame(can_frame message);
  std::optional<can_frame> receiveCanFrame();

 private:
  int socket_;
  hyped::core::ILogger &logger_;
};
}  // namespace hyped::io

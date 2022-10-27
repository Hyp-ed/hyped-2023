#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {
class Can {
 public:
  Can(hyped::core::ILogger &logger);
  int sendCanFrame(can_frame message);
  can_frame receiveCanFrame();

 private:
  int socket_;
  hyped::core::ILogger &logger_;
};
}  // namespace hyped::io

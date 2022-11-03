#pragma once

namespace hyped::motorcontrollers {
class Controller {
 public:
  Controller();
  hyped::core::ILogger &logger_;
};
}  // namespace hyped::motorcontrollers
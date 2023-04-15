#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include <core/logger.hpp>
#include <core/time.hpp>
#include <core/wall_clock.hpp>
#include <state_machine/message.hpp>
#include <state_machine/state_machine.hpp>

class transitionpub : public rclcpp::Node {
 public:
  transitionpub(hyped::core::ILogger &logger) : Node("transitionpub"), logger_(logger)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_     = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&transitionpub::timer_callback, this));
  }

 private:
  void timer_callback()
  {
    // changes of the state due to handle message will be published back to the topic here
    auto message = std_msgs::msg::String();
    message.data = "kCalibrating";
    logger_.log(hyped::core::LogLevel::kInfo, "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  hyped::core::ILogger &logger_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  hyped::core::WallClock time;
  hyped::core::Logger logger("Topic", hyped::core::LogLevel::kDebug, time);

  std::shared_ptr pub = std::make_shared<transitionpub>(logger);

  rclcpp::executors::StaticSingleThreadedExecutor executor;

  executor.add_node(pub);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
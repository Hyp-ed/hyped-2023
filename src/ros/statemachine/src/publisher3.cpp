#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <core/logger.hpp>
#include <core/time.hpp>
#include <core/wall_clock.hpp>

class Stm3 : public rclcpp::Node {
 public:
  Stm3(hyped::core::ILogger &logger) : Node("stm3"), count_(0), logger_(logger)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_     = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&Stm3::timer_callback, this));
  }

 private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hi! " + std::to_string(count_);
    logger_.log(hyped::core::LogLevel::kInfo, "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    ++count_;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::size_t count_;
  hyped::core::ILogger &logger_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  hyped::core::WallClock time;
  hyped::core::Logger logger("Topic", hyped::core::LogLevel::kDebug, time);
  rclcpp::spin(std::make_shared<Stm3>(logger));
  rclcpp::shutdown();
  return 0;
}
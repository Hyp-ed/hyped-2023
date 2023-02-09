#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include <core/logger.hpp>
#include <core/time.hpp>
#include <core/wall_clock.hpp>

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher(hyped::core::ILogger &logger)
      : Node("random_data_publisher"),
        minimum_(0),
        maximum_(10),
        count_(0),
        logger_(logger)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int16>("random", 10);
    timer_     = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int16();
    message.data = rand() % (maximum_ - minimum_ + 1) + minimum_;
    if (count_ == 10) {
      count_ = 0;
      maximum_ += 10;
      minimum_ += 10;
    }
    logger_.log(hyped::core::LogLevel::kInfo, "Publishing: '%d'", message.data);
    publisher_->publish(message);
    ++count_;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
  std::uint16_t minimum_;
  std::uint16_t maximum_;
  std::size_t count_;
  hyped::core::ILogger &logger_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  hyped::core::WallClock time;
  hyped::core::Logger logger("Random", hyped::core::LogLevel::kDebug, time);
  rclcpp::spin(std::make_shared<MinimalPublisher>(logger));
  rclcpp::shutdown();
  return 0;
}


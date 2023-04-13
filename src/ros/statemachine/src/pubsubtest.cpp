#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include <core/logger.hpp>
#include <core/time.hpp>
#include <core/wall_clock.hpp>

class Stm1sub : public rclcpp::Node
{
  public:
    Stm1sub()
    : Node("stm1")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&Stm1sub::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class Stm1pub : public rclcpp::Node{
    public:
     Stm1pub(hyped::core::ILogger &logger)
     : Node("stm1"),
       count_(0),
       logger_(logger)
    {
        publisher_ = this ->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_     = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&Stm1pub::timer_callback, this));
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  hyped::core::WallClock time;
  hyped::core::Logger logger("Topic", hyped::core::LogLevel::kDebug, time);

  std::shared_ptr pub = std::make_shared<Stm1pub>(logger);
  std::shared_ptr sub = std::make_shared<Stm1sub>();

  rclcpp::executors::StaticSingleThreadedExecutor executor;

  executor.add_node(pub);
  executor.add_node(sub);

  //rclcpp::spin(std::make_shared<Stm1>());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
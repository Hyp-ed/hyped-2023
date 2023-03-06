#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <core/logger.hpp>
#include <core/time.hpp>
#include <core/wall_clock.hpp>

class MinimalPublisher : public rclcpp::Node{
    public:
    MinimalPublisher()
        : Node("publisher"), 
            count_(0)
    
    public:
     MinimalPublisher()
     : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this ->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_     = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&MinimalPublisher::timer_callback, this));
    }

    private:
     void timer_callback()
     {
        auto message = std_msgs::msg::String();
        message.data = "Hi! " + std::to_string(count_);
        logger_.log(hyped::core::LogLevel::kInfo, "Publishing: '%d'", message.data);
    publisher_->publish(message);
    ++count_;
     }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    hyped::core::ILogger &logger_;

};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    hyped::core::WallClock time;
    hyped::Logger logger("Topic", hyped::core::LogLevel::kDebug, time);
    rclcpp::spin(std::make_shared<MinimalPublisher>(logger));
    rclcpp::shutdown();
    return 0;
}
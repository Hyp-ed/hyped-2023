#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("csv_logger"),
      count_(0)
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int16>(
      "random", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      file_.open("data.csv");
      file_ << "count, data" << std::endl;
    }

  private:
    void topic_callback(const std_msgs::msg::Int16::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
      file_ << count_ << ", " << msg->data << std::endl;
      ++count_;
    }
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
    std::ofstream file_;
    uint16_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
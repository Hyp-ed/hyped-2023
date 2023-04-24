#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include <core/logger.hpp>
#include <core/time.hpp>
#include <core/wall_clock.hpp>
#include <state_machine/message.hpp>
#include <state_machine/state_machine.hpp>

class Stm3pub : public rclcpp::Node {
 public:
  Stm3pub(hyped::core::ILogger &logger, std::unique_ptr<hyped::state_machine::StateMachine> &stm3)
      : Node("stm3pub"),
        logger_(logger),
        stm3_(stm3)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_     = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&Stm3pub::timer_callback, this));
  }

 private:
  void timer_callback()
  {
    // changes of the state due to handle message will be published back to the topic here
    auto message = std_msgs::msg::String();
    message.data = stm3_->messageToString(stm3_->getPreviousMessage());
    // logger_.log(hyped::core::LogLevel::kInfo, "Made Transition: '%s'", message.data.c_str());
    RCLCPP_INFO(this->get_logger(), "Made Transition: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  hyped::core::ILogger &logger_;
  std::unique_ptr<hyped::state_machine::StateMachine> &stm3_;
};

class Stm3sub : public rclcpp::Node {
 public:
  Stm3sub(std::unique_ptr<hyped::state_machine::StateMachine> &stm3, std::shared_ptr<Stm3pub> &pub)
      : Node("stm3sub"),
        stm3_(stm3),
        pub_(pub)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&Stm3sub::topic_callback, this, _1));
  }

 private:
  void topic_callback(const std_msgs::msg::String &msg) const
  {
    // this is where we will hear new messages and this is where we will call handleMessage
    hyped::state_machine::Message message = stm3_->stringToMessage(msg.data);
    bool transitioned                     = stm3_->handleMessage(message);
    // we only publish a message if a transition was made
    if (transitioned) {
      rclcpp::spin_some(pub_);
    } else {
      stm3_->getPreviousMessage();
    }
    // RCLCPP_INFO(this->get_logger(), "New Message: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::unique_ptr<hyped::state_machine::StateMachine> &stm3_;
  std::shared_ptr<Stm3pub> &pub_;
};

int main(int argc, char *argv[])
{
  std::unique_ptr stm3 = std::make_unique<hyped::state_machine::StateMachine>();

  rclcpp::init(argc, argv);

  hyped::core::WallClock time;
  hyped::core::Logger logger("Topic", hyped::core::LogLevel::kDebug, time);

  std::shared_ptr pub3 = std::make_shared<Stm3pub>(logger, stm3);
  std::shared_ptr sub3 = std::make_shared<Stm3sub>(stm3, pub3);

  rclcpp::executors::StaticSingleThreadedExecutor executor3;

  // executor.add_node(pub);
  executor3.add_node(sub3);

  executor3.spin();

  rclcpp::shutdown();
  return 0;
}
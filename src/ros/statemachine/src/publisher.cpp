// include part

class MinimalPublisher : public rclcpp::Node{
    public:
    MinimalPublisher()
        : Node("publisher1"), 
            count_(0)
    {
        publisher_ = this ->create_puclisher<std_msgs::msg::String>
    }
}
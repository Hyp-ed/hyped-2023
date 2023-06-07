import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class ROSListener(Node):
    def __init__(self):
        super().__init__('ros_listener')

        self.subscription_1 = self.create_subscription(
            String,
            'mqtt1',
            self.callback,
            10
        )

        self.subscription_2 = self.create_subscription(
            Float32,
            'mqtt2',
            self.callback,
            10
        )

    def callback(self, msg):
        # Callback function to process received messages
        self.get_logger().info(f'ROS Listener: Received message: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    simple_listener = ROSListener()
    rclpy.spin(simple_listener)

    simple_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

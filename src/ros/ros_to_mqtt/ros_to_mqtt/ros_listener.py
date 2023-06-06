import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleListener(Node):
    def __init__(self):
        super().__init__('simple_listener')

        # Subscribe to the desired topic
        self.subscription1 = self.create_subscription(
            String,  # Replace with the appropriate message type
            'mqtt1',  # Replace with the desired topic name
            self.callback,
            10
        )
        self.subscription1

        self.subscription2 = self.create_subscription(
            String,  # Replace with the appropriate message type
            'mqtt2',  # Replace with the desired topic name
            self.callback,
            10
        )
        self.subscription2

    def callback(self, msg):
        # Callback function to process received messages
        self.get_logger().info(f'ROS LISTENER: Received message: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    simple_listener = SimpleListener()
    rclpy.spin(simple_listener)

    simple_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

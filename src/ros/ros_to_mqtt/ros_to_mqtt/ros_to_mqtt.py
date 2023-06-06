import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log


class RosoutSubscriber(Node):
    def __init__(self):
        super().__init__('rosout_subscriber')

        self.create_subscription(
            Log,
            'rosout',
            self.rosout_callback,
            10
        )

    def rosout_callback(self, msg):
        # Process the received rosout message here
        # We don't want to log the rosout_subscriber's own messages (or we'll get an infinite loop)
        if (msg.name == "rosout_subscriber"):
            return
        self.log(msg)

    def log(self, msg):
        self.get_logger().info(f"ROSOUT: {msg.level}: {msg.name}: {msg.msg}")


def main(args=None):
    rclpy.init(args=args)
    node = RosoutSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

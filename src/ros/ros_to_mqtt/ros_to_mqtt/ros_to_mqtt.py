import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import paho.mqtt.client as mqtt


class RosoutSubscriber(Node):
    def __init__(self):
        super().__init__('rosout_subscriber')

        self.create_subscription(
            Log,
            'rosout',
            self.rosout_callback,
            10
        )

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.connect("localhost", 1883, 60)

    def rosout_callback(self, msg):
        # Process the received rosout message here
        # We don't want to log the rosout_subscriber's own messages (or we'll get an infinite loop)
        if (msg.name == "rosout_subscriber"):
            return
        self.log(msg)
        self.client.publish(
            "from_rosout", f"{msg.level}: {msg.name}: {msg.msg}")

    def log(self, msg):
        self.get_logger().info(f"ROSOUT: {msg}")

    def on_connect(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))


def main(args=None):
    rclpy.init(args=args)
    node = RosoutSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

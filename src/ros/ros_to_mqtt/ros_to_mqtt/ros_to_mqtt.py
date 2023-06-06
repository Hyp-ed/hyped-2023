import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt


class RosToMqttBridge(Node):
    def __init__(self, config):
        super().__init__('ros_to_mqtt_bridge')
        self.mqtt_client = None
        self.subscription_list = []

        # MQTT connection configuration
        mqtt_config = config['mqtt']['connection']
        self.mqtt_host = mqtt_config['host']
        self.mqtt_port = mqtt_config['port']
        self.mqtt_keepalive = mqtt_config['keepalive']

        # ROS topics configuration
        topics_config = config['topics']

        # Connect to the MQTT broker
        self.connect_mqtt()

        # Subscribe to ROS topics and publish to MQTT
        for topic_config in topics_config:
            topic_name = topic_config['topic_name']
            message_type = topic_config['message_type']
            mqtt_topic = topic_config['mqtt_topic']
            callback_function = getattr(
                self, topic_config['function'])

            self.subscription_list.append(self.create_subscription(
                msg_type=String,  # Replace with the appropriate message type
                topic=topic_name,
                callback=self.create_callback(callback_function, mqtt_topic),
                qos_profile=rclpy.qos.qos_profile_sensor_data
            ))

    def connect_mqtt(self):
        # Connect to the MQTT broker
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(
            self.mqtt_host, self.mqtt_port, self.mqtt_keepalive)
        self.mqtt_client.loop_start()

    def create_callback(self, callback_function, topic_name):
        def callback_wrapper(msg):
            # This wrapper function allows passing the topic name to the callback
            msg = callback_function(msg, topic_name)
            self.mqtt_client.publish(topic_name, msg.data)

        return callback_wrapper

    def callback1(self, msg, topic_name):
        # Callback function for topic1
        self.get_logger().info(f'Received message on topic1: {msg.data}')
        # Do some processing on msg here
        msg.data = msg.data + ' processed'
        return msg

    def callback2(self, msg, topic_name):
        # Callback function for topic2
        self.get_logger().info(f'Received message on topic2: {msg.data}')
        # Do some processing on msg here
        return msg


def main(args=None):
    rclpy.init(args=args)

    # Read the configuration from the file
    import yaml
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    ros_to_mqtt_bridge = RosToMqttBridge(config)
    rclpy.spin(ros_to_mqtt_bridge)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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
        ros_to_mqtt_topic_config = config['ros-to-mqtt']

        # Connect to the MQTT broker
        self.connect_mqtt()

        # Subscribe to ROS topics and publish to MQTT
        for topic_config in ros_to_mqtt_topic_config:
            topic_name = topic_config['ros_topic_name']
            message_type = topic_config['message_type']
            mqtt_topic = topic_config['mqtt_topic_name']
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
        self.get_logger().info(
            f'Received message on {topic_name} (callback1): {msg.data}')
        # Do some processing on msg here
        msg.data = msg.data + ' processed'
        return msg

    def callback2(self, msg, topic_name):
        # Callback function for topic2
        self.get_logger().info(
            f'Received message on {topic_name} (callback2): {msg.data}')
        # Do some processing on msg here
        return msg


class MqttToRosBridge(Node):
    def __init__(self, config):
        super().__init__('mqtt_to_ros_bridge')
        self.mqtt_client = None

        # MQTT connection configuration
        mqtt_config = config['mqtt']['connection']
        self.mqtt_host = mqtt_config['host']
        self.mqtt_port = mqtt_config['port']
        self.mqtt_keepalive = mqtt_config['keepalive']

        # ROS topics configuration
        mqtt_to_ros_topic_config = config['mqtt-to-ros']

        # Connect to the MQTT broker
        self.connect_mqtt()

        # def on_message(client, userdata, message):
        #     print(f"message received {message.payload.decode('utf-8')}")

        # self.mqtt_client.on_message = on_message

        # self.mqtt_client.on_message = self.create_callback(mqtt_to_ros_topic_config["function"])

        for topic_config in mqtt_to_ros_topic_config:
            topic_name = topic_config['mqtt_topic_name']
            message_type = topic_config['message_type']
            mqtt_topic = topic_config['ros_topic_name']
            callback_function = getattr(
                self, topic_config['function'])

            self.mqtt_client.message_callback_add(
                topic_name, self.create_callback(callback_function, mqtt_topic))

        # Subscribe to MQTT topics by passing an array of topic names
        self.mqtt_client.subscribe(
            [(topic_config['mqtt_topic_name'], 0) for topic_config in mqtt_to_ros_topic_config])

    def connect_mqtt(self):
        # Connect to the MQTT broker
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(
            self.mqtt_host, self.mqtt_port, self.mqtt_keepalive)
        self.mqtt_client.loop_start()

    def create_callback(self, callback_function, topic_name):
        def callback_wrapper(client, userdata, message):
            # This wrapper function allows passing the topic name to the callback
            msg = callback_function(message, topic_name)
            # Publish to ROS
            msg1 = String()
            msg1.data = msg
            self.create_publisher(
                String, topic_name, 10).publish(msg1)

        return callback_wrapper

    def callback3(self, msg, topic_name):
        # Callback function for topic1
        # self.get_logger().info(
        #     f'Received message on {topic_name} (callback3): {msg.data}')
        # Do some processing on msg here
        # msg.data = msg.data + ' processed'
        print(msg.payload.decode('utf-8'))
        return msg.payload.decode('utf-8')

    def callback4(self, msg, topic_name):
        # Callback function for topic2
        # self.get_logger().info(
        #     f'Received message on {topic_name} (callback4): {msg.data}')
        # Do some processing on msg here
        print(msg.payload.decode('utf-8'))
        return msg.payload.decode('utf-8')


def main(args=None):
    rclpy.init(args=args)

    # Read the configuration from the file
    import yaml
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    ros_to_mqtt_bridge = RosToMqttBridge(config)
    mqtt_to_ros_bridge = MqttToRosBridge(config)
    rclpy.spin(ros_to_mqtt_bridge)
    rclpy.spin(mqtt_to_ros_bridge)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

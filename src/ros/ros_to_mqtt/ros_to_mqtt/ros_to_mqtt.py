import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
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

        self.mqtt_qos = config["mqtt"]["qos"]

        # ROS topics configuration
        ros_to_mqtt_topic_config = config['ros-to-mqtt']

        # Connect to the MQTT broker
        self.connect_mqtt()

        # Subscribe to ROS topics and publish to MQTT
        for topic_config in ros_to_mqtt_topic_config:
            ros_topic_name = topic_config['ros_topic_name']
            message_type = topic_config['message_type']
            mqtt_topic_name = topic_config['mqtt_topic_name']
            callback_function = getattr(
                self, topic_config['function'])

            self.subscription_list.append(self.create_subscription(
                msg_type=Float32 if message_type == "float" else String,
                topic=ros_topic_name,
                callback=self.create_callback(
                    callback_function, mqtt_topic_name),
                qos_profile=rclpy.qos.qos_profile_sensor_data
                # TODO: I think we can use callback_args here to pass the mqtt_topic_name without using a wrapper function
            ))

    def connect_mqtt(self):
        """ Connects to the MQTT broker """
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(
            self.mqtt_host, self.mqtt_port, self.mqtt_keepalive)
        self.mqtt_client.loop_start()

    def create_callback(self, callback_function, topic_name):
        """ Create a callback function for the subscription """
        def callback_wrapper(msg):
            """ This wrapper function allows passing the topic name to the callback and sharing code between the callbacks """
            msg = callback_function(msg, topic_name)
            # Publish to MQTT
            self.mqtt_client.publish(topic_name, msg.data, self.mqtt_qos)

        return callback_wrapper

    def string_callback(self, msg, topic_name):
        """ Callback function for topic1 """
        self.get_logger().info(
            f'Received ROS message on {topic_name} (string_callback): {msg.data}')
        # Do some processing on msg here
        msg.data = msg.data + ' processed by callback1'
        return msg

    def float_callback(self, msg, topic_name):
        """ Callback function for topic2 """
        self.get_logger().info(
            f'Received ROS message on {topic_name} (float_callback): {str(msg.data)}')
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

        # Add message callbacks for MQTT topics
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
        """ Create a callback function for the subscription """
        def callback_wrapper(client, userdata, message):
            """ 
            This wrapper function allows sharing code between the callbacks
            (we don't need to pass the topic names because the MQTT message already has this information)
            """
            mqtt_msg = callback_function(message)
            # Publish to ROS
            ros_msg = String()
            ros_msg.data = mqtt_msg.payload.decode("utf-8")
            self.create_publisher(
                String, topic_name, 10).publish(ros_msg)

        return callback_wrapper

    def callback3(self, msg):
        """ Callback function for topic1. Receives a Paho MQTT message object and returns a Paho MQTT message object """
        self.get_logger().info(
            f'Received MQTT message on {msg.topic} (callback3): {msg.payload.decode("utf-8")}')
        # Do some processing on msg here
        msg.payload = msg.payload + b' processed by callback3'
        return msg

    def callback4(self, msg):
        """ Callback function for topic2 """
        self.get_logger().info(
            f'Received MQTT message on {msg.topic} (callback4): {msg.payload.decode("utf-8")}')
        # Do some processing on msg here
        return msg


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

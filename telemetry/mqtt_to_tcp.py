import paho.mqtt.client as mqtt
import socket
import time
import json

# Connect to TCP server
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect(("localhost", 65432))

# Connect to MQTT broker and subscribe to all topics with message callback
mqttc = mqtt.Client()
mqttc.connect("localhost", 1883, 60)
mqttc.subscribe("#", 0)


def on_message(client, userdata, msg):
    """ Callback function for MQTT messages """
    print("Received message on topic: " + msg.topic)
    print("Message: " + msg.payload.decode("utf-8"))

    # Build the JSON message
    json_msg = {
        "topic": msg.topic,
        "value": msg.payload.decode("utf-8"),
        "timestamp": time.time()
    }

    # Send the message
    # s.sendall(json.dumps(json_msg).encode("utf-8"))


# Assign callback function to receive messages
mqttc.on_message = on_message

# Loop forever, receiving messages
mqttc.loop_forever()

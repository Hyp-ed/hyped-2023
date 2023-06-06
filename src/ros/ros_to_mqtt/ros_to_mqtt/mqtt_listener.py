import paho.mqtt.client as mqtt
import yaml


def on_connect(client, userdata, flags, rc):
    """ The callback for when the client receives a CONNACK response from the server. """
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.

    # subscribe to all topics
    client.subscribe("#")


def on_message(client, userdata, msg):
    """ The callback for when a PUBLISH message is received from the server."""
    print(f"MQTT Listener: {msg.topic} {str(msg.payload)}")


# Create an MQTT client
client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
with open("config.yaml", "r") as config_file:
    config = yaml.safe_load(config_file)
    client.connect(config["mqtt"]["connection"]["host"],
                   config["mqtt"]["connection"]["port"], config["mqtt"]["connection"]["keepalive"])

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()

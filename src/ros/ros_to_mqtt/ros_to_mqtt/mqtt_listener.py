import paho.mqtt.client as mqtt
import yaml


def on_connect(client, userdata, flags, rc):
    """ The callback for when the client receives a CONNACK response from the server. """
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.

    client.subscribe("#")
    print("Subscribed to all topics")


def on_message(client, userdata, msg):
    """ The callback for when a PUBLISH message is received from the server."""
    print(
        f"MQTT Listener: Receieved message: {str(msg.payload)} on topic {msg.topic}")


# Create an MQTT client
client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message

with open("config.yaml", "r") as config_file:
    config = yaml.safe_load(config_file)

# Connect to the MQTT broker using the configuration
client.connect(config["mqtt"]["connection"]["host"],
               config["mqtt"]["connection"]["port"], config["mqtt"]["connection"]["keepalive"])

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()

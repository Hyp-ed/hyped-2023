import paho.mqtt.client as mqtt
import time
import yaml

# Create an MQTT client
client = mqtt.Client()

with open("config.yaml", "r") as config_file:
    config = yaml.safe_load(config_file)

mqtt_qos = config["mqtt"]["qos"]

# Connect to the MQTT broker
client.connect(config["mqtt"]["connection"]["host"],
               config["mqtt"]["connection"]["port"], config["mqtt"]["connection"]["keepalive"])

# Publish messages to the topic indefinitely
i = 0
while True:
    client.publish("mqtt1", "Hello from MQTT mqtt1! No." + str(i), mqtt_qos)
    print("MQTT PUBLISH: Hello from MQTT mqtt1! No." + str(i))

    client.publish("mqtt2", float(i), mqtt_qos)
    print("MQTT PUBLISH: " + str(i))

    time.sleep(1)
    i += 1

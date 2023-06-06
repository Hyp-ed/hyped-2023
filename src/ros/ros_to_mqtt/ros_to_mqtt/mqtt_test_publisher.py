import paho.mqtt.client as mqtt
import time
import yaml

# Create an MQTT client
client = mqtt.Client()

# Connect to the MQTT broker
with open("config.yaml", "r") as config_file:
    config = yaml.safe_load(config_file)
    client.connect(config["mqtt"]["connection"]["host"],
                   config["mqtt"]["connection"]["port"], config["mqtt"]["connection"]["keepalive"])

mqtt_qos = config["mqtt"]["qos"]
i = 0

# Publish messages to the topic indefinitely
while True:
    client.publish("mqtt1", "Hello from MQTT mqtt1! No." + str(i), mqtt_qos)
    print("MQTT PUBLISH: Hello from MQTT mqtt1! No." + str(i))

    client.publish("mqtt2", "Hello from MQTT mqtt2! No." + str(i), mqtt_qos)
    print("MQTT PUBLISH: Hello from MQTT mqtt2! No." + str(i))

    time.sleep(1)
    i += 1

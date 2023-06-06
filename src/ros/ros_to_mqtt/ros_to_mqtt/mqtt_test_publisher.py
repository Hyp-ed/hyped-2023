import paho.mqtt.client as mqtt
import time

# Create an MQTT client
client = mqtt.Client()

# Connect to the MQTT broker
client.connect('localhost', 1883, 60)

i = 0

# Publish messages to the topic indefinitely
while True:
    client.publish("mqtt1", "Hello from MQTT mqtt1!" + str(i))
    print("MQTT PUBLISH: Hello from MQTT mqtt1!" + str(i))
    client.publish("mqtt2", "Hello from MQTT mqtt2!" + str(i))
    print("MQTT PUBLISH: Hello from MQTT mqtt2!" + str(i))
    time.sleep(1)  # Adjust the sleep duration as needed
    i += 1

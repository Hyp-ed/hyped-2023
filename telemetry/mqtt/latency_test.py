import paho.mqtt.client as mqtt

# Connect to MQTT broker and subscribe to all topics with message callback
mqttc = mqtt.Client()
mqttc.connect("localhost", 1883, 60)
mqttc.subscribe("#", 0)


def on_message(client, userdata, msg):
    if msg.topic == "hyped/pod_1/latency/request":
        print("Received latency request" + str(msg.payload))
        mqttc.publish("hyped/pod_1/latency/response", msg.payload)
        print("Sent latency response")


# Assign callback function to receive messages
mqttc.on_message = on_message

# Loop forever, receiving messages
mqttc.loop_forever()

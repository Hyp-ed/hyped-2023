import paho.mqtt.client as mqtt
import socket
import time
import json

CLIENT_HOST = "192.168.1.40"
CLIENT_PORT = 65432

# Connect to TCP server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((CLIENT_HOST, CLIENT_PORT))

conn = s.listen(1)
# Wait for 2 connections
connections = []
while True:
    conn, addr = s.accept()
    print("Connected by", addr)
    connections.append(conn)
    if len(connections) == 1:
        break

# Connect to MQTT broker and subscribe to all topics with message callback
mqttc = mqtt.Client()
mqttc.connect("localhost", 1883, 60)
mqttc.subscribe("#", 0)


def on_message(client, userdata, msg):
    # print("onMessage")
    """ Callback function for MQTT messages """
    # Ignore messages from the pod
    if msg.topic.startswith("hyped/pod_1/measurement/") or msg.topic == "hyped/pod_1/latency/response":
        # print("Ignoring message from pod")
        return
    # Handle latency
    if msg.topic == "hyped/pod_1/latency/request":
        # print("Received latency request")
        # Send latency request over TCP
        # for conn in connections:
            # if msg.payload == None:
                # print(msg)
            # conn.sendall(msg.payload)
        return
    print("Received message on topic: " + msg.topic)
    # print("Message: " + msg.payload.encode())

    # Build the JSON message
    json_msg = {
        "topic": msg.topic,
        "value": msg.payload,
        "timestamp": time.time(),
    }

    # Send the message over TCP
    # conn.sendall(json.dumps(json_msg).encode("utf-8"))
    for conn in connections:
        print(json_msg)
        conn.sendall(json_msg["value"])


# Assign callback function to receive messages
mqttc.on_message = on_message

# Loop forever, receiving messages
mqttc.loop_forever()

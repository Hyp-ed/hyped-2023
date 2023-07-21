import paho.mqtt.client as mqtt
import socket
import json
import time

# get local machine name
HOST = "192.168.1.40"
PORT = 65433

# create a socket to receive data from the TCP sender
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

mqtt_client = mqtt.Client()

# connect to MQTT broker
mqtt_client.connect("localhost", 1883, 60)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    # Wait for multiple connections
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            # loop through the dictionary and publish each key-value pair
            for key, value in json.loads(data).items():
                print(data)
                if key == "latency":
                    print("Sending latency response" + str(value))
                    mqtt_client.publish("hyped/pod_1/latency/response", value)
                elif key.startswith("accelerometer"):
                    mqtt_client.publish(
                        "hyped/pod_1/measurement/" + key, value["x"])
                elif key.startswith("keyence_one"):
                    mqtt_client.publish(
                        "hyped/pod_1/measurement/keyence_1", value["num_stripes_detected"])
                elif key.startswith("keyence_two"):
                    mqtt_client.publish(
                        "hyped/pod_1/measurement/keyence_2", value["num_stripes_detected"])
                else:
                    mqtt_client.publish(
                        f'hyped/pod_1/measurement/{key}', value)
                print(f"Published {key}: {value}")

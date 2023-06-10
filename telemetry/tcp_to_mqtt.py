import paho.mqtt.client as mqtt
import socket
import json

# create a socket to receive data from the TCP sender
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# get local machine name
HOST = "127.0.0.1"
PORT = 65432

mqtt_client = mqtt.Client()

# connect to MQTT broker
mqtt_client.connect("localhost", 1883, 60)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            # loop through the dictionary and publish each key-value pair
            for key, value in json.loads(data).items():
                mqtt_client.publish(f'hyped/pod_1/measurement/{key}', value)
                print(f"Published {key}: {value}")

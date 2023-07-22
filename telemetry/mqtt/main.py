# main.py combines tcp_to_mqtt, mqtt_to_tcp, and the heartbeat into one file.

import paho.mqtt.client as mqtt
import socket
import time
import json
import threading

# PI_IP = "192.168.1.56"
PI_IP = "localhost"
POD_ID = "pod_1"
# number of connections to expect
NUM_CONNECTIONS = 1


def mqtt_connection():
    """ Creates an MQTT client, connects to the broker, and subscribes to all topics """
    mqtt_client = mqtt.Client()
    mqtt_client.connect("localhost", 1883, 60)
    mqtt_client.subscribe("#", 0)
    return mqtt_client


def tcp_to_mqtt():
    """ Converts TCP messages to MQTT messages (pod -> GUI) """
    print("STARTING: TCP -> MQTT")

    TCP_PORT = 65433

    # Connect to MQTT broker
    mqtt_client = mqtt_connection()

    while True:
        try:
            # Create a socket to receive data from the TCP sender
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Bind the socket to the desired address and port
            s.bind((PI_IP, TCP_PORT))
            s.listen()

            # Wait for a connection
            conn, addr = s.accept()
            print(f"[TCP -> MQTT] Connected by {addr}")

            while True:
                try:
                    data = conn.recv(1024)
                    if not data:
                        break
                    # Loop through the dictionary and publish each key-value pair
                    for key, value in json.loads(data).items():
                        if key.startswith("accelerometer"):
                            mqtt_client.publish(
                                "hyped/pod_1/measurement/" + key, value["x"])
                        elif key.startswith("keyence_1"):
                            mqtt_client.publish(
                                "hyped/pod_1/measurement/keyence_1", value["num_stripes_detected"])
                        elif key.startswith("keyence_2"):
                            mqtt_client.publish(
                                "hyped/pod_1/measurement/keyence_2", value["num_stripes_detected"])
                        else:
                            mqtt_client.publish(
                                f'hyped/pod_1/measurement/{key}', value)
                        print(f"[TCP -> MQTT] MQTT Published {key}: {value}")

                except:
                    print("Connection with TCP sender lost. Retrying...")
                    break

        except socket.error as e:
            print(f"Socket error: {e}")
            time.sleep(2)


def mqtt_to_tcp():
    """ Converts MQTT messages to TCP messages (GUI -> pod) """
    print("STARTING: MQTT -> TCP")

    TCP_PORT = 65432

    # Connect to TCP server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((PI_IP, TCP_PORT))

    conn = s.listen(1)

    # Wait for NUM_CONNECTIONS connections
    connections = []
    while True:
        conn, addr = s.accept()
        print("[MQTT -> TCP] Connected by", addr)
        connections.append(conn)
        if len(connections) == NUM_CONNECTIONS:
            break

    mqtt_client = mqtt_connection()

    def on_message(client, userdata, msg):
        """ Callback function for MQTT messages """
        print("on message")

        # Ignore messages from the pod
        if msg.topic.startswith(f"hyped/{POD_ID}/measurement") or msg.topic == f"hyped/{POD_ID}/latency/response":
            return

        # HEARTBEAT: Handle latency requests
        if msg.topic == f"hyped/{POD_ID}/latency/request":
            mqtt_client.publish(
                f"hyped/{POD_ID}/latency/response", msg.payload)
            return

        json_msg = {
            "topic": msg.topic,
            "value": msg.payload,
            "timestamp": time.time(),
        }

        # Send the message over TCP
        for conn in connections:
            conn.sendall(json_msg['value'])
            print(
                f"[MQTT -> TCP] TCP Sent {json_msg['topic']}: {json_msg['value']}")

    mqtt_client.on_message = on_message
    mqtt_client.loop_forever()


if __name__ == "__main__":
    try:
        thread1 = threading.Thread(target=tcp_to_mqtt)
        thread2 = threading.Thread(target=mqtt_to_tcp)

        # Set the threads as daemon threads so they automatically exit when the main thread exits
        thread1.daemon = True
        thread2.daemon = True

        # Start the threads
        thread1.start()
        thread2.start()

        # Keep the main thread running to respond to KeyboardInterrupt (Ctrl+C)
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting...")
        exit(0)

# TCP sender

import socket
import json
import time
import random

MIN_PRESSURE = 3.26
MAX_PRESSURE = 4.6

# create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# get local machine name
host = "localhost"
port = 65433

# connection to hostname on the port.
s.connect((host, port))

while True:
    s.send(json.dumps({
        "hyped/pod_1/measurement/pressure": random.uniform(MIN_PRESSURE, MAX_PRESSURE),
    }).encode('utf-8'))
    print("Sent")
    time.sleep(1)

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
port = 65432

# connection to hostname on the port.

s.connect((host, port))

while True:
    s.send(json.dumps({
        "pressure_1": random.uniform(MIN_PRESSURE, MAX_PRESSURE),
        "pressure_2": random.uniform(MIN_PRESSURE, MAX_PRESSURE),
        "pressure_3": random.uniform(MIN_PRESSURE, MAX_PRESSURE),
        "pressure_4": random.uniform(MIN_PRESSURE, MAX_PRESSURE),
        "pressure_5": random.uniform(MIN_PRESSURE, MAX_PRESSURE),
        "pressure_6": random.uniform(MIN_PRESSURE, MAX_PRESSURE),
        "pressure_7": random.uniform(MIN_PRESSURE, MAX_PRESSURE),
        "pressure_8": random.uniform(MIN_PRESSURE, MAX_PRESSURE),
    }).encode('utf-8'))
    time.sleep(0.5)

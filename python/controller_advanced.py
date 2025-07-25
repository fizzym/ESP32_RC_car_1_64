#! /usr/bin/env python3

import serial
import time
import re

MAX_BUFF_LEN = 255
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
MAX_RETRIES = 5

connected = False
port = None

def read_serial(num_char = 1):
    return port.read(num_char).decode()


def write_serial(cmd):
    cmd = cmd + '\n'
    port.write(cmd.encode())


# Connecting to ESP32
connections_tried = 1
prev = time.time()
while (not connected):
    try:
        port = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    except:
        if (time.time() - prev > 2):
            print("Can't connect to serial port: {}. Retrying {}".
                  format(SERIAL_PORT, connections_tried))
            prev = time.time()
            connections_tried += 1
            
        # Only try to connect for a set number of attempts
        if connections_tried > MAX_RETRIES:
            exit(2)
    if (port is not None):
        print("Connected to {}".format(SERIAL_PORT))
        connected = True

# Sending commands to ESP32
while (True):
    '''
    Command format is direction (forward, backward) strength steering 
    (left, right) strength

    For example:
     'f2l0' : forward 2; left 0 (i.e. straight)
     'b3r2' : backward 3; right 2
    '''
    command = input("Enter command of the form (f3l2, b1r2, s5s5): ")

    # Sanitize the command:
    regexp = re.compile(r"^[fbs]\d[lrs]\d$")
    if regexp.search(command):
        write_serial(command)
        print("Sent command: {}".format(command))
        reply = read_serial(MAX_BUFF_LEN)
        if (len(reply) > 0):
            print(reply)
    else:
        print("Unrecognized command: {}".format(command))
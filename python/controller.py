#! /usr/bin/env python3

import serial
import time

MAX_BUFF_LEN = 255
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

connected = False
port = None


def read_serial(num_char = 1):
    return port.read(num_char).decode()


def write_serial(cmd):
    port.write(cmd)


prev = time.time()
while (not connected):
    try:
        port = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    except:
        if (time.time() - prev > 2):
            print("Can't connect to serial port: {}".format(SERIAL_PORT))
            prev = time.time()
    if (port is not None):
        connected = True


while (True):
    command = (input("Enter command (f, b, l, r, s): ")).encode()
    if command in [x.encode() for x in ['f', 'b', 'l', 'r', 's']]:
        write_serial(command)
        print("Sent command: {}".format(command))
    else:
        print("Unrecognized command: {}".format(command))
    
    reply = read_serial(MAX_BUFF_LEN)
    if (len(reply) > 0):
        print(reply)
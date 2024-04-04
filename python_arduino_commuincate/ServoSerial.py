#!/usr/bin/env python3

import serial
import time
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
from pathlib import Path

MAX_BUFF_LEN = 255
SETUP = False
port = None

prev = time.time()
while not SETUP:
    try:
        # Serial port(windows-->COM), baud rate, timeout msg
        port = serial.Serial("COM6", 115200, timeout=1)

    except Exception as e:
        if time.time() - prev > 2:  # Don't spam with msg
            print("No serial detected, please plug in your microcontroller")
            prev = time.time()

    if port is not None:  # We're connected
        SETUP = True

# Read one char (default)
def read_ser(num_char=1):
    string = port.read(num_char)
    return string.decode()

# Write whole strings
def write_ser(cmd):
    cmd = cmd + '\n'
    port.write(cmd.encode())

# Super loop
while True:
    string = read_ser(MAX_BUFF_LEN)
    if len(string):
        print(string)

    cmd = input("Enter command ('on' or 'off' or servo angle): ").lower()  # Blocking, there're solutions for this ;)
    if cmd:
        write_ser(cmd)

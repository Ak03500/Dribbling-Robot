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


# Calibration parameters (you need to adjust these based on your setup)
pixels_per_cm = 10  # Adjust this based on your calibration
z_scale = 1.0  # Adjust this based on your calibration

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
    help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
pts = deque(maxlen=args["buffer"])

# Servo control states
SWEEP_STATE = 1
STOP_STATE = 0
servo_state = STOP_STATE

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    vs = VideoStream(src=0).start()

# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

# keep looping
while True:
    # grab the current frame
    frame = vs.read()

    # handle the frame from VideoCapture or VideoStream
    frame = frame[1] if args.get("video", False) else frame

    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
        break

    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # Calculate the x, y, and z coordinates based on the mapping
            x_real = (center[0]) / pixels_per_cm
            y_real = (center[1]) / pixels_per_cm
            z_real = radius * z_scale

            # Display coordinates on the frame
            cv2.putText(frame, f"X: {x_real:.2f} cm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Y: {y_real:.2f} cm", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Z: {z_real:.2f} cm", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
             # Check x position to control the servo
            if x_real > 25:
                 # Send a command to ESP32 to make the servo sweep
                cmd = "sweep\n"
                try:
                    port.write(cmd.encode())
                    port.flush()
                except Exception as e:
                    print(f"Error writing to serial port: {e}")
            else:
                 # Send a command to ESP32 to stop the servo
                cmd = "stop\n"
                try:
                    port.write(cmd.encode())
                    port.flush()
                except Exception as e:
                    print(f"Error writing to serial port: {e}")
               

    # update the points queue
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
       # Send servo command to the ESP32
    if servo_state == SWEEP_STATE:
        write_ser("sweep")
    else:
        write_ser("stop")
# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
    vs.stop()

# otherwise, release the camera
else:
    vs.release()

# close all windows
cv2.destroyAllWindows()

import serial
import time
from collections import deque
#from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
#import imutils
import time
from pathlib import Path
from pid import PID
# Create PID controllers for x and y axes
pid_x = PID(kp=0.1, ki=0.01, kd=0.1)
pid_y = PID(kp=0.1, ki=0.01, kd=0.1)

# Set the desired setpoint to the center of the platform
pid_x.set_setpoint(19)  # assuming center is at 0 for x-axis
pid_y.set_setpoint(19)  # assuming center is at 0 for y-axis

# def example(angle):
#     return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US

def map_pid_output_to_pwm(pid_output):
    # Example conversion: 1500 µs is the center position of the servo.
    # Assume PID output range from -10 to 10 needs to map to 1400 µs to 1600 µs
    center_pwm = 1500
    scale = 10  # This scale factor would be adjusted based on your specific hardware setup
    pwm_signal = center_pwm + pid_output * scale
    return max(1200, min(1700, pwm_signal))  # Ensure PWM signal stays within servo range


MAX_BUFF_LEN = 255

SETUP = False
port = None
counter = 0
prev = time.time()
while not SETUP:
    try:
        # Serial port(windows-->COM), baud rate, timeout msg
        port = serial.Serial("COM6", 460800, timeout=1)

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
greenLower = (50, 100,100)
greenUpper = (80, 255, 255)
pts = deque(maxlen=args["buffer"])

# Servo control states
SWEEP_STATE = 1
STOP_STATE = 0
servo_state = STOP_STATE

# if a video path was not supplied, grab the reference
# to the webcam
# if not args.get("video", False):
#     vs = VideoStream(src=0).start()

# # otherwise, grab a reference to the video file
# else:
#     vs = cv2.VideoCapture(args["video"])

vs = cv2.VideoCapture(0, cv2.CAP_DSHOW) #captureDevice = camera
# allow the camera or video file to warm up
time.sleep(2.0)
prevtime = time.time()
# keep looping
while True:
    # grab the current frame
    ret, frame = vs.read()
    if not ret:
        break  # Break the loop if unable to grab a frame

    # Resize the frame
    frame = cv2.resize(frame, (600, int(frame.shape[0] * 600 / frame.shape[1])))
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
        break

    # resize the frame, blur it, and convert it to the HSV
    # color space
    # frame = cv2.resize(frame, width=600)
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
    # cnts = imutils.grab_contours(cnts)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
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

            # Display coordinates on the frameqqq
            cv2.putText(frame, f"X: {x_real:.2f} cm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Y: {y_real:.2f} cm", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Z: {z_real:.2f} cm", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            counter += 1

            currentTime = time.time()
            controlX = pid_x.update(x_real, currentTime)
            controlY = pid_y.update(y_real, currentTime)

            pwmX = map_pid_output_to_pwm(-controlX)
            pwmY = map_pid_output_to_pwm(-controlY)
            
            # command = f"PWMX:{pwmX},PWMY:{pwmY}\n"
            # port.write(command.encode('utf-8'))
            # print("Sent to ESP32:", command.strip())


            #Check the x position to control the servo based on the threshold
            # if x_real > 24:
            #     x = 9
            #     y = 0
            # else:
            #     x = 0
            #     y = 1
            command = f"{int(pwmX)},{int(pwmY)}\n"  # Format the string with both values
            port.write(command.encode('utf-8'))  # Send the command
            print(command)

            # # time.sleep(0.1)


            # if x_real > 18:
            #      # Send a command to ESP32 to make the servo sweep
            #     cmd = "1"
            #     port.write(cmd.encode())
            #     print(cmd)
            # else:
            #      # Send a command to ESP32 to stop the servo
            #      cmd = "0"
            #      port.write(cmd.encode())
            #      print(cmd)


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


#print(counter)
# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
    vs.release()

# otherwise, release the camera
else:
    vs.release()

# close all windows
cv2.destroyAllWindows()


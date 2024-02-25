import cv2
import numpy as np
import time

# Function to create a Kalman Filter
def create_kalman_filter():
    kalman = cv2.KalmanFilter(6, 3)  # 6 states (x, y, z, vx, vy, vz), 3 measurements (x, y, z)
    kalman.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0],
                                         [0, 1, 0, 0, 0, 0],
                                         [0, 0, 1, 0, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 0, 1, 0, 0],
                                        [0, 1, 0, 0, 1, 0],
                                        [0, 0, 1, 0, 0, 1],
                                        [0, 0, 0, 1, 0, 0],
                                        [0, 0, 0, 0, 1, 0],
                                        [0, 0, 0, 0, 0, 1]], np.float32)
    kalman.processNoiseCov = 1e-5 * np.eye(6, dtype=np.float32)
    kalman.measurementNoiseCov = 1e-1 * np.eye(3, dtype=np.float32)
    return kalman

cap = cv2.VideoCapture(0)

# Known diameter of the orange ball in centimeters
known_diameter_cm = 4.0

# Focal length (example value, update based on your camera calibration)
focal_length = 1000.0

# Initialize Kalman Filter
kalman = create_kalman_filter()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([0, 100, 100])
    upper_orange = np.array([30, 255, 255])

    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_area = 100
    max_area = 50000  # Increase this value to allow for larger contours

    for contour in contours:
        area = cv2.contourArea(contour)

        if min_area < area < max_area:
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w // 2
            cy = y + h // 2

            # Calculate the apparent diameter of the ball in pixels
            apparent_diameter = max(w, h)

            # Calculate the depth (z-coordinate) using the metric depth estimation formula
            depth = (known_diameter_cm * focal_length) / apparent_diameter

            # Predict the next state of the Kalman Filter
            kalman.predict()

            # Update the Kalman Filter with measurements
            kalman.correct(np.array([[cx], [cy], [depth]], dtype=np.float32))

            # Get filtered state (estimated position and velocity)
            state = kalman.statePost
            ema_x, ema_y, ema_z, vel_x, vel_y, vel_z = state.flatten()

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (int(ema_x), int(ema_y)), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"X: {int(ema_x)} Y: {int(ema_y)} Z: {int(ema_z)} cm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"VX: {int(vel_x)} VY: {int(vel_y)} VZ: {int(vel_z)} cm/s", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Ball Tracking with Kalman Filter", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

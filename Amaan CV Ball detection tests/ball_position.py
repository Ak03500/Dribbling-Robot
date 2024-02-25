import cv2
import numpy as np

# Capture video from the default camera (you may need to change the index)
cap = cv2.VideoCapture(0)

# Known diameter of the orange ball in centimeters
known_diameter_cm = 4.0

# Focal length (example value, update based on your camera calibration)
focal_length = 1000.0

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    # Apply Gaussian blur to the frame
    blurred = cv2.GaussianBlur(frame, (15, 15), 0)

    # Convert the blurred frame to the HSV color space
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the orange color
    lower_orange = np.array([0, 100, 100])
    upper_orange = np.array([30, 255, 255])

    # Create a mask using the inRange function
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate through the contours
    for contour in contours:
        # Calculate the area and perimeter of the contour
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        # Set a minimum and maximum area to filter out small and large contours
        min_area = 100
        max_area = 10000

        # Check if the perimeter is zero to avoid division by zero
        if perimeter == 0:
            continue

        # Calculate circularity
        circularity = 4 * np.pi * (area / (perimeter * perimeter))

        if min_area < area < max_area and circularity > 0.5:
            # Find the bounding rectangle around the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Calculate the center of the rectangle
            cx = x + w // 2
            cy = y + h // 2

            # Calculate the apparent diameter of the ball in pixels
            apparent_diameter = max(w, h)

            # Calculate the depth (z-coordinate) using the metric depth estimation formula
            depth = (known_diameter_cm * focal_length) / apparent_diameter

            # Draw a rectangle around the object on the original frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Draw a circle at the center of the rectangle on the original frame
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # Display the x, y, and z coordinates at the top left of the frame
            cv2.putText(frame, f"X: {int(cx)} Y: {int(cy)} Z: {int(depth)} cm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the original frame with the rectangle and coordinates drawn
    cv2.imshow("Frame", frame)

    # Break the loop when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()

import numpy as np
import cv2
import serial
import time


try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    print("Serial connection established. Using external camera.")
    camera_index = 0  # External camera index if available.
except serial.SerialException:
    print("Serial connection not found. Defaulting to macOS's webcam.")
    ser = None
    camera_index = 1  # Default webcam if serial connection fails.


# Initialize Kalman Filter for smoothing the face position.
# 4 state variables: x, y, vx, vy; 2 measurements: x, y.
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                      [0, 1, 0, 1],
                                      [0, 0, 1, 0],
                                      [0, 0, 0, 1]], np.float32)
# Process noise covariance: affects how aggressively the filter tracks motion.
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03


# Load the face detection cascade.
face_cascade = cv2.CascadeClassifier("data/haarcascade_frontalface_default.xml")
cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set height

# Get frame dimensions and compute the frame center.
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
center_x, center_y = frame_width // 2, frame_height // 2


# Parameters for Lucas-Kanade optical flow.
lk_params = dict(winSize=(25, 25),
                 maxLevel=3,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Initialize variables for tracking
prev_gray = None       # Previous grayscale image for optical flow calculation.
prev_points = None     # Previous points to track (face center).
face_detected = False
valid_tracking = False  # This will be True only if the face was successfully detected in the last detection cycle.


# Instead of a fixed frame count, use a time-based interval.
detection_interval = 1  # In seconds; forces re-detection if tracking continues for this long.
last_detection_time = time.time()  # Timestamp of the last detection.


face_box_dims = None   # Stores width and height of the detected face bounding box.
last_face_center = None  # Stores the last detected face center position.


# To avoid sending commands every frame, track the last command sent.
last_command = None
command_cooldown = 0.2  # Minimum time between commands in seconds.
last_command_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    current_time = time.time()  # Current timestamp for time-based checks.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to grayscale for detection and optical flow.
    

    # If we have lost tracking or enough time has passed, run face detection.
    if not valid_tracking or (current_time - last_detection_time > detection_interval):
        # Downscale the grayscale image to speed up detection by reducing resolution.
        scale_factor = 0.99  # Scale down to 50% of the original resolution.
        small_gray = cv2.resize(gray, (0, 0), fx=scale_factor, fy=scale_factor)
        
        # Run face detection on the downscaled image.
        faces = face_cascade.detectMultiScale(small_gray, scaleFactor=1.1, minNeighbors=10)
        
        if len(faces) > 0:
            # If a face is detected, scale the coordinates back to the original image size.
            x, y, w, h = faces[0]
            x = int(x / scale_factor)
            y = int(y / scale_factor)
            w = int(w / scale_factor)
            h = int(h / scale_factor)
            
            # Calculate the center of the detected face.
            face_center = (x + w // 2, y + h // 2)
            face_box_dims = (w, h)  # Save the dimensions for drawing the bounding box later.
            last_face_center = face_center

            # Initialize the Kalman filter with the detected face center.
            state = np.array([[face_center[0]],
                              [face_center[1]],
                              [0],
                              [0]], np.float32)
            kalman.statePre = state.copy()
            kalman.statePost = state.copy()
            face_detected = True
            valid_tracking = True

            # Set up optical flow: use the detected face center as the initial point to track.
            prev_gray = gray.copy()
            prev_points = np.array([[face_center]], dtype=np.float32)
        else:
            # If no face is detected, mark tracking as invalid.
            face_detected = False
            valid_tracking = False

        # Update the last detection timestamp.
        last_detection_time = current_time


    # If valid tracking is established, use optical flow to update the face position.
        # This is when the optical flow algorithm successfully found the new location of the previously detected face center.
    # When tracking is valid (i.e., status[0][0] == 1), you use the new position from next_points to update the Kalman filter. 
        # This predicted position is then used to draw the bounding box, ensuring that the box continues to follow the face smoothly.
    elif valid_tracking and prev_gray is not None and prev_points is not None:
        # Calculate the new position of the tracked point using Lucas-Kanade method.
        # LK Method is computationally efficient b/c it only tracks lk_params['winSize'] pixels around the point.
        # https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
        next_points, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_points, None, **lk_params)
        if status is not None and status[0][0] == 1:
            # If optical flow successfully tracks the point, use it as the new measurement.
            measured = np.array([[next_points[0][0][0]],
                                 [next_points[0][0][1]]], np.float32)
            kalman.correct(measured)  # Correct Kalman filter state with this new measurement.
            
            # Update the last known face center:
            # - Convert the floating point coordinates from the optical flow measurement to integers.
            # - Store these coordinates in the `last_face_center` variable.
            #
            # This variable represents the most recent center position of the detected face.
            # It is used for:
            #   • Providing visual feedback (e.g., drawing an accurately positioned bounding box).
            #   • Serving as a fallback value if subsequent tracking steps fail.
            last_face_center = (int(measured[0][0]), int(measured[1][0]))
        else:
            # If tracking fails (e.g., due to occlusion), mark tracking as lost.
            valid_tracking = False

        # Update previous frame and points for the next optical flow calculation.
        prev_gray = gray.copy()
        prev_points = next_points


    # Use the Kalman filter to predict the next position.
    prediction = kalman.predict()
    pred_x, pred_y = int(prediction[0]), int(prediction[1])
    
    # Compute the offset between the predicted face center and the frame center.
    dx = center_x - pred_x
    dy = center_y - pred_y


    if valid_tracking and face_box_dims is not None:
        # Draw a line from the predicted face center to the frame center.
        cv2.line(frame, (pred_x, pred_y), (center_x, center_y), (255, 255, 0), 2)

        # Use the last detected face dimensions to draw a bounding box around the predicted face.
        w, h = face_box_dims
        top_left = (pred_x - w // 2, pred_y - h // 2)
        bottom_right = (pred_x + w // 2, pred_y + h // 2)

        # Change the bounding box color: green if the horizontal offset is small, red otherwise.
        box_color = (0, 255, 0) if abs(dx) < 50 else (0, 0, 255)
        cv2.rectangle(frame, top_left, bottom_right, box_color, 1)

        # Display the offset values on the frame.
        cv2.putText(frame, f"dx: {dx}, dy: {dy}", (frame_width - 220, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


        # Send serial commands only if the horizontal offset exceeds a threshold.
        # Use throttling to avoid flooding the controller with too many commands.
        if ser is not None:
            # Define tolerance
            tolerance = 20

            # Determine the direction for dx and dy
            dx_command = "NONE"  # Default
            if abs(dx) >= tolerance:
                dx_command = "RIGHT" if dx > 0 else "LEFT"

            dy_command = "NONE"  # Default
            if abs(dy) >= tolerance:
                dy_command = "DOWN" if dy < 0 else "UP"

            # Handle the message combinations
            if dx_command == "NONE" and dy_command == "NONE":
                # Both dx and dy are within tolerance, no message needed
                pass
            elif dx_command == "NONE" or dy_command == "NONE":
                # Only one direction is out of tolerance, send the respective message
                command = f"{dx_command},{dy_command}\n"
                ser.write(command.encode())
            else:
                # Both dx and dy are out of tolerance, send the combined message
                command = f"{dx_command},{dy_command}\n"
                ser.write(command.encode())

    # Display the tracking frame.
    cv2.imshow('Face Tracking', frame)
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up and release resources.
cap.release()
if ser is not None:
    ser.close()
cv2.destroyAllWindows()
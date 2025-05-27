import cv2
import pygame
import numpy as np
import math
import time  # Import time for debouncing
import serial

# Initialize pygame for controller input
pygame.init()
pygame.joystick.init()

# Check if a controller is connected
if pygame.joystick.get_count() == 0:
    print("No controller detected!")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# Initialize OpenCV video capture
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Screen properties
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Crosshair properties
dot_x, dot_y = width // 2, height // 2
speed = 10  # Adjust speed of movement
thickness = 1  # Line thickness
gap = 25  # Gap in the center

# Define the rotation angle (45 degrees)
angle = 45
theta = np.radians(angle)  # Convert angle to radians
cos_theta = np.cos(theta)
sin_theta = np.sin(theta)

show_dotted_line = True

# Variables for tracking and color
display_data = True
is_tracking = False
crosshair_color = (67, 164, 40)  # Green
dotted_line_color = (45, 240, 88)  # Green (for moving crosshair)
font = cv2.FONT_HERSHEY_SIMPLEX

# Helper function to rotate points around a center
def rotate_point(x, y, cx, cy, angle):
    radians = math.radians(angle)
    cos_a = math.cos(radians)
    sin_a = math.sin(radians)
    
    # Translate point to origin
    x -= cx
    y -= cy
    
    # Rotate and translate back
    new_x = cx + (x * cos_a - y * sin_a)
    new_y = cy + (x * sin_a + y * cos_a)
    
    return int(round(new_x)), int(round(new_y))


# Function to draw the static crosshair
def draw_crosshair(frame, crosshair_color):
    cross_size = 20  # Length of each stroke
    cx, cy = width // 2, height // 2
    
    # Draw the center outer circle
    cv2.circle(frame, (cx, cy), 50, crosshair_color, 1, cv2.LINE_AA)

    # Draw the center inner circle
    cv2.circle(frame, (cx, cy), 1, crosshair_color, 1, cv2.LINE_AA)
    
    
    # Vertical and Horizontal lines
    top_left = (cx, cy - cross_size)
    top_right = (cx, cy - gap)
    bottom_left = (cx, cy + gap)
    bottom_right = (cx, cy + cross_size)

    left_top = (cx - cross_size, cy)
    left_bottom = (cx - gap, cy)
    right_top = (cx + gap, cy)
    right_bottom = (cx + cross_size, cy)
    
    # Rotate points
    top_left_rot = rotate_point(top_left[0], top_left[1], cx, cy, angle)
    top_right_rot = rotate_point(top_right[0], top_right[1], cx, cy, angle)
    bottom_left_rot = rotate_point(bottom_left[0], bottom_left[1], cx, cy, angle)
    bottom_right_rot = rotate_point(bottom_right[0], bottom_right[1], cx, cy, angle)

    left_top_rot = rotate_point(left_top[0], left_top[1], cx, cy, angle)
    left_bottom_rot = rotate_point(left_bottom[0], left_bottom[1], cx, cy, angle)
    right_top_rot = rotate_point(right_top[0], right_top[1], cx, cy, angle)
    right_bottom_rot = rotate_point(right_bottom[0], right_bottom[1], cx, cy, angle)

    # Draw lines
    cv2.line(frame, top_left, top_right, crosshair_color, thickness, cv2.LINE_8)
    cv2.line(frame, bottom_left, bottom_right, crosshair_color, thickness, cv2.LINE_8)
    cv2.line(frame, left_top, left_bottom, crosshair_color, thickness, cv2.LINE_8)
    cv2.line(frame, right_top, right_bottom, crosshair_color, thickness, cv2.LINE_8)

    # Draw rotated lines
    cv2.line(frame, top_left_rot, top_right_rot, crosshair_color, thickness, cv2.LINE_8)
    cv2.line(frame, bottom_left_rot, bottom_right_rot, crosshair_color, thickness, cv2.LINE_8)
    cv2.line(frame, left_top_rot, left_bottom_rot, crosshair_color, thickness, cv2.LINE_8)
    cv2.line(frame, right_top_rot, right_bottom_rot, crosshair_color, thickness, cv2.LINE_8)

# Function to draw the moving crosshair
def draw_moving_crosshair(frame, dot_x, dot_y, color, angle):
    cross_size = 10
    circle_radius = 15  # Circle size behind the crosshair
    
    # Apply sobel circle filter
    apply_edge_detection_filter(frame, dot_x, dot_y, circle_radius, color)

     # Apply vivid light circle filter
    #apply_vivid_light_filter(frame, dot_x, dot_y, circle_radius)

    # Define crosshair line endpoints **before** rotation
    half_size = cross_size // 2  # Ensures symmetry
    points = [
        (dot_x, dot_y - half_size),  # Top
        (dot_x, dot_y + half_size),  # Bottom
        (dot_x - half_size, dot_y),  # Left
        (dot_x + half_size, dot_y)   # Right
    ]
    
    # Rotate points around the center (dot_x, dot_y)
    rotated_points = [rotate_point(p[0], p[1], dot_x, dot_y, angle) for p in points]
    
    # Draw crosshair lines
    cv2.line(frame, rotated_points[0], rotated_points[1], color, 1, cv2.LINE_AA)
    cv2.line(frame, rotated_points[2], rotated_points[3], color, 1, cv2.LINE_AA)

    # Draw the center limit circle
    draw_multiply_circle(frame, dot_x, dot_y, color)

def draw_multiply_circle(frame, dot_x, dot_y, color):
    """Add a thin circle outline with Multiply blending mode to the given frame."""
    radius = 15  # Fixed radius
    mask = np.zeros_like(frame, dtype=np.uint8)
    
    # Draw the hollow circle outline on the mask (thickness 1)
    cv2.circle(mask, (dot_x, dot_y), radius, color, 1, cv2.LINE_AA)

    # Convert to float for Multiply blending
    frame_float = frame.astype(np.float32) / 255
    mask_float = mask.astype(np.float32) / 255

    # Apply Multiply blending and add to the original frame
    blended = frame_float * mask_float

    # Convert back to 8-bit and add to the frame
    blended = (blended * 255).astype(np.uint8)
    frame[:] = cv2.add(frame, blended)  # Modify the frame in place

# Edge detection filter
def apply_edge_detection_filter(frame, dot_x, dot_y, radius, color):
    """Apply edge detection filter (Sobel filter) to the circular area of the given radius."""
    
    # Convert the whole frame to grayscale first for edge detection
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Sobel filter to detect edges (horizontal and vertical gradients)
    grad_x = cv2.Sobel(gray_frame, cv2.CV_64F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray_frame, cv2.CV_64F, 0, 1, ksize=3)

    # Compute gradient magnitude
    grad_mag = cv2.magnitude(grad_x, grad_y)

    # Normalize gradient magnitude for visualization
    grad_mag = np.clip(grad_mag, 0, 255)
    grad_mag = grad_mag.astype(np.uint8)

    # Use a kernel for morphological operations (erosion to thin edges)
    kernel = np.ones((4, 4), np.uint8)  # A smaller kernel for thinner edges
    grad_mag = cv2.erode(grad_mag, kernel, iterations=1)  # Erosion makes edges thinner

    #Inverse color
    inverse_color = tuple(map(lambda i, j: abs(i - j), (255,255,255), color))

    # Create the circle mask and focus on the area within the circle
    for y in range(dot_y - radius, dot_y + radius):
        for x in range(dot_x - radius, dot_x + radius):
            # Check if the point is inside the circle
            if (x - dot_x) ** 2 + (y - dot_y) ** 2 <= radius ** 2:
                if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                    # If the gradient magnitude is above a threshold, highlight the edge
                    if grad_mag[y, x] > 100:  # Threshold to detect edges
                        # Draw a small line at the edge location
                        frame[y, x] = list(inverse_color)  # Highlight edge in red
                    else:
                        # Keep original color for non-edge areas
                        frame[y, x] = frame[y, x]

#Vivid light filter
def apply_vivid_light_filter(frame, dot_x, dot_y, radius):
    """Apply vivid light filter to the circular area of the given radius, adjusting contrast visibility."""
    
    # Convert the whole frame to grayscale first for edge detection
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Sobel filter to detect edges (horizontal and vertical gradients)
    grad_x = cv2.Sobel(gray_frame, cv2.CV_64F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray_frame, cv2.CV_64F, 0, 1, ksize=3)

    # Compute gradient magnitude
    grad_mag = cv2.magnitude(grad_x, grad_y)

    # Normalize gradient magnitude for visualization
    grad_mag = np.clip(grad_mag, 0, 255)
    grad_mag = grad_mag.astype(np.uint8)

    # Use a kernel for morphological operations (erosion to thin edges)
    kernel = np.ones((3, 3), np.uint8)  # A smaller kernel for thinner edges
    grad_mag = cv2.erode(grad_mag, kernel, iterations=1)  # Erosion makes edges thinner

    # Create the circle mask and focus on the area within the circle
    for y in range(dot_y - radius, dot_y + radius):
        for x in range(dot_x - radius, dot_x + radius):
            # Check if the point is inside the circle
            if (x - dot_x) ** 2 + (y - dot_y) ** 2 <= radius ** 2:
                if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                    # If the gradient magnitude is above a threshold, invert the color of the pixel
                    if grad_mag[y, x] > 100:  # Threshold to detect edges
                        # Invert the color of the pixel (color inversion)
                        frame[y, x] = 255 - frame[y, x]
                    
                    # Apply vivid light filter for contrast adjustment
                    pixel = frame[y, x].copy()
                    modified_pixel = pixel.copy()
                    for i in range(3):  # For BGR channels
                        value = pixel[i]
                        
                        # Adjust contrast based on the pixel value
                        if value < 128:
                            # Darken the pixel (similar to linear burn)
                            modified_pixel[i] = value - (255 - value) * 0.25
                        else:
                            # Lighten the pixel (similar to linear dodge)
                            modified_pixel[i] = value + (255 - value) * 0.25
                        
                        # Clip the new value to ensure it's within valid bounds
                        modified_pixel[i] = np.clip(modified_pixel[i], 0, 255)
                    
                    # Blend the original and modified pixel based on intensity
                    frame[y, x] = np.clip((1 - 0.25) * pixel + 0.25 * modified_pixel, 0, 255)


# Function to draw dotted line
def draw_dotted_line(frame, cx, cy, dot_x, dot_y, line_color):
    length = int(np.sqrt((dot_x - cx)**2 + (dot_y - cy)**2))
    step = 4
    for i in range(0, length, step * 2):
        x = int(cx + i * (dot_x - cx) / length)
        y = int(cy + i * (dot_y - cy) / length)
        cv2.circle(frame, (x, y), 0, line_color, -1)

# Display the tracking variables
def display_crosshair_info(frame, d, dx, dy, color):
    # Calculate the angle in degrees and convert to 0-360 range
    angle = math.degrees(math.atan2(dy, dx))
    if angle < 0:
        angle += 360  # Convert negative angles to positive

    # Display the distance and angle
    text = f"(d:{d:.0f}, a:{angle:.0f})"

    # Get the size of the text to center it
    (text_width, text_height), _ = cv2.getTextSize(text, font, 0.45, 0)

    # Calculate the starting x position to center the text horizontally
    start_x = (width - text_width) // 2

    # Draw the text centered horizontally and placed below the center vertically
    cv2.putText(frame, text, (start_x, height // 2 + 65), font, 0.45, color, 0, cv2.LINE_8)

def draw_tracking_text(frame, color):
    text = "TRACKING"
    
    # Get text size
    (text_width, text_height), _ = cv2.getTextSize(text, font, 0.40, 0)
    

    # Calculate the starting x position to center the text horizontally
    start_x = (width - text_width) // 2

    if(display_data):
        # Draw the text centered horizontally and placed below the center vertically
        cv2.putText(frame, text, (start_x, height // 2 + 65 + 15), font, 0.40, color, 0, cv2.LINE_8)
    else:
        # Draw the text centered horizontally and placed below the center vertically
        cv2.putText(frame, text, (start_x, height // 2 + 65), font, 0.40, color, 0, cv2.LINE_8)

# Debouncing for button press
last_button_press_time = 0
debounce_interval = 0.5  # seconds

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    print("Serial connection established. Using external camera.")
except serial.SerialException:
    print("Serial connection not found. Defaulting to macOS's webcam.")
    ser = None

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame = cv2.flip(frame, 1)
    
    pygame.event.pump()
    x_axis = joystick.get_axis(3)
    y_axis = joystick.get_axis(4)

    # Check for "Menu" button press to toggle tracking
    if joystick.get_button(6):  # Button 6 is the menu button
        current_time = time.time()
        if current_time - last_button_press_time > debounce_interval:
            display_data = not display_data
            last_button_press_time = current_time

    # Check for "A" button press with debounce
    if joystick.get_button(0):  # Button 0 is the A button
        current_time = time.time()
        if current_time - last_button_press_time > debounce_interval:
            if crosshair_color == (67, 164, 40):  # Green
                crosshair_color = (0, 0, 255)  # Red
                dotted_line_color = (0, 0, 255)  # Light red
            else:
                crosshair_color = (67, 164, 40)  # Green
                dotted_line_color = (45, 240, 88)  # Green
            last_button_press_time = current_time  # Update the last press time

    # Check for left bumper press (button index 4)
    if joystick.get_button(4):
        current_time = time.time()
        if current_time - last_button_press_time > debounce_interval:
            is_tracking = not is_tracking
            last_button_press_time = current_time    
    
    if joystick.get_button(4):
        current_time = time.time()
        if current_time - last_button_press_time > debounce_interval:
            is_tracking = not is_tracking
            last_button_press_time = current_time

    # Check right trigger (axis 5)
    if joystick.get_axis(5) > 0.5:  # Adjust threshold if needed
        joystick.rumble(0.5, 0.5, 500)  # Rumble at 50% strength for 500ms
    else:
        joystick.rumble(0, 0, 0)  # Stop rumble

    # Reset crosshair to center with middle joystick button
    if joystick.get_button(10):
        dot_x, dot_y = width // 2, height // 2

    # Update crosshair position
    dot_x += int(x_axis * speed)
    dot_y += int(y_axis * speed)

    # Keep crosshair within screen bounds
    dot_x = max(0, min(width - 1, dot_x))
    dot_y = max(0, min(height - 1, dot_y))

    # Calculate distance and components from center
    dx = dot_x - width // 2
    dy = dot_y - height // 2
    d = np.sqrt(dx**2 + dy**2)

    # Draw the crosshair and dotted line
    draw_crosshair(frame, crosshair_color)
    draw_moving_crosshair(frame, dot_x, dot_y, dotted_line_color, 0)
    if show_dotted_line:
        draw_dotted_line(frame, width // 2, height // 2, dot_x, dot_y, dotted_line_color)

    # Display tracking info if enabled
    if display_data:
        display_crosshair_info(frame, d, dx, dy, crosshair_color)

    if is_tracking:
        draw_tracking_text(frame, crosshair_color)
    
    # Send serial commands only if the horizontal offset exceeds a threshold.
    # Use throttling to avoid flooding the controller with too many commands.
    if ser is not None and is_tracking:
        # Define tolerance
        tolerance = 20

        # Determine the direction for dx and dy
        dx_command = "NONE"  # Default
        if abs(dx) >= tolerance:
            dx_command = "RIGHT" if dx > 0 else "LEFT"

        dy_command = "NONE"  # Default
        if abs(dy) >= tolerance:
            dy_command = "DOWN" if dy > 0 else "UP"

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



    # Show the frame
    cv2.imshow('Webcam Control', frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
pygame.quit()

import cv2
import numpy as np
from . import motor_control

# Global variables for mouse position
mouse_x = -1
mouse_y = -1

# Get mouse position
def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x = x
        mouse_y = y

# Checks if box a and box b overlap
def boxes_overlap(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b

    return not (
        ax + aw < bx or
        bx + bw < ax or
        ay + ah < by or
        by + bh < ay
    )

# Creates video output and commands motors
def generate_frames(cap, robot):
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Red detection 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Hue / Saturation / Brightness ranges
        lower_red1 = np.array([0, robot.color_tolerences[1], robot.color_tolerences[2]])
        upper_red1 = np.array([robot.color_tolerences[0], 255, 255])

        lower_red2 = np.array([180-robot.color_tolerences[0], robot.color_tolerences[1], robot.color_tolerences[2]])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Clean mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find individual objects
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # Create list of potential objects
        rectangles = []
        for cnt in contours:
            # Rectangular bounding box
            x, y, w, h = cv2.boundingRect(cnt)

            if w*h < robot.area_min:
                continue

            # Must fit reasonable aspect ratio range, unless cut off by FOV
            if y > 2 and x > 2 and x < robot.video_width-2:
                aspect_ratio = h / float(w)
                if aspect_ratio > 1.50+robot.ratio_tolerence or aspect_ratio < 1.50-robot.ratio_tolerence:
                    continue
            
            rectangles.append((x, y, w, h))

        closest = (640, 0, 0, 0)
        # Check for overlap, remove smaller boxes if present
        for rect in rectangles:
            keep = True
            for ref_rect in rectangles:
                if boxes_overlap(rect, ref_rect):
                    if rect[2]*rect[3] < ref_rect[2]*ref_rect[3]:
                        keep = False
                        break

            if keep:
                if rect[1]>closest[1]:
                    closest = rect
                # Draw lowest point
                lowest_point = (int(rect[0] + rect[2]/2), rect[1] + rect[3])
                cv2.circle(frame, lowest_point, 8, (255, 0, 0), -1)

                # Draw bounding box
                cv2.rectangle(frame, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
        
        closest_point = (int(closest[0] + closest[2]/2), closest[1] + closest[3])

        # Direct motors to go to point (if enabled)
        if robot.motor_action:
            turn_factor, distance = motor_control.go_to(closest_point[0], closest_point[1], robot)
            # Show driving variables on video
            cv2.putText(frame, f"Turn Factor: {turn_factor:.4f}", (760, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Distance: {distance}", (760, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Display HSV values at mouse position
        if mouse_x >= 0 and mouse_y >= 0 and mouse_x < robot.video_width and mouse_y < robot.video_height:
            h, s, v = hsv[mouse_y, mouse_x]
            text = f"H:{h} S:{s} V:{v}"
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Webcam", frame)

        # Process GUI events and allow window to update
        key = cv2.waitKey(1) & 0xFF

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Start can detection and following
def begin_tracking(robot):

    # Open and set up webcam
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, robot.video_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, robot.video_height)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) # Enable image compression
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Set to manual exposure mode
    cap.set(cv2.CAP_PROP_EXPOSURE, -7)  # Set exposure time to 2^-7 = 1/128 second
    cap.set(cv2.CAP_PROP_AUTO_WB, 0.0) # Disable auto white balance
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4200) # Set white balance temperature to 4200K
    cap.set(cv2.CAP_PROP_FPS, 30) # Set frames per second

    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")

    # Create window and set mouse callback
    cv2.namedWindow("Webcam")
    cv2.setMouseCallback("Webcam", mouse_callback)

    # Begin
    generate_frames(cap, robot)

    # Cleanup (at the end)
    cap.release()
    cv2.destroyAllWindows()
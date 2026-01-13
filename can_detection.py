import cv2
import numpy as np
from flask import Flask, Response
import motor_control

# Open webcam
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) # Enable image compression
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Set to manual exposure mode
cap.set(cv2.CAP_PROP_EXPOSURE, -7)  # Set exposure time to 2^-7 = 1/128 second
cap.set(cv2.CAP_PROP_AUTO_WB, 0.0) # Disable auto white balance
cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4200) # Set white balance temperature to 4200K
cap.set(cv2.CAP_PROP_FPS, 30) # Set frames per second

if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

def boxes_overlap(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b

    return not (
        ax + aw < bx or
        bx + bw < ax or
        ay + ah < by or
        by + bh < ay
    )

def generate_frames():
    closest = (720, 360, 0, 0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Red detection 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Hue / Saturation / Brightness ranges
        lower_red1 = np.array([0, 110, 70])
        upper_red1 = np.array([5, 255, 255])

        lower_red2 = np.array([175, 110, 70])
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

        rectangles = []
        for cnt in contours:
            RATIO_TOLERENCE = 0.1
            # Rectangular bounding box
            x, y, w, h = cv2.boundingRect(cnt)

            if w*h < 5000:
                continue

            # Must fit reasonable aspect ratio range, unless cut off by FOV
            if y > 2 and x > 2 and x < 1278:
                aspect_ratio = h / float(w)
                if aspect_ratio > 1.50+RATIO_TOLERENCE or aspect_ratio < 1.50-RATIO_TOLERENCE:
                    continue
            
            rectangles.append((x, y, w, h))

        # Check for overlap, remove smaller boxes if present
        for rect in rectangles:
            print(f"Width: {rect[2]}, Height: {rect[3]}")
            keep = True
            for ref_rect in rectangles:
                if boxes_overlap(rect, ref_rect):
                    if rect[2]*rect[3] < ref_rect[2]*ref_rect[3]:
                        keep = False
                        break

            if keep:
                if rect[0]<closest[0]:
                    closest = rect
                # Draw lowest point
                lowest_point = (int(rect[0] + rect[2]/2), rect[1] + rect[3])
                cv2.circle(frame, lowest_point, 8, (255, 0, 0), -1)

                # Draw bounding box
                cv2.rectangle(frame, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
        
        closest_point = (int(closest[0] + closest[2]/2), closest[1] + closest[3])
        # motor_control.go_to(closest_point[0], closest_point[1])

        cv2.imshow("Webcam", frame)
        # Process GUI events and allow window to update
        key = cv2.waitKey(1) & 0xFF
        # Press 'q' or ESC to quit
        if key == ord('q') or key == 27:
            break

if __name__ == "__main__":
    generate_frames()
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
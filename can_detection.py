import cv2
import numpy as np
from flask import Flask, Response

app = Flask(__name__)

# Open webcam
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

def generate_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Red detection 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 170, 120])
        upper_red1 = np.array([5, 255, 255])

        lower_red2 = np.array([175, 170, 120])
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

        for cnt in contours:
            RATIO_TOLERENCE = 0.1
            # Rectangular bounding box
            x, y, w, h = cv2.boundingRect(cnt)

            area = cv2.contourArea(cnt)
            if area < 500:
                continue

            # Must fit reasonable aspect ratio range
            aspect_ratio = h / float(w)
            if aspect_ratio > 1.50+RATIO_TOLERENCE or aspect_ratio < 1.50-RATIO_TOLERENCE:
                continue

            # Draw lowest point
            lowest_point = (int(x + w/2), y + h)
            cv2.circle(frame, lowest_point, 8, (255, 0, 0), -1)

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Encode frame as JPEG
        _, buffer = cv2.imencode(".jpg", frame)
        frame_bytes = buffer.tobytes()

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" +
               frame_bytes + b"\r\n")

@app.route("/")
def video_feed():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)

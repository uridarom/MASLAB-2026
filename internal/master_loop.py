from internal.MaslabRobot import MaslabRobot
from internal import video
from robot import motor_control
from raven import Raven
import cv2
import numpy as np

# Global variables for mouse position
mouse_x = -1
mouse_y = -1
raven = Raven()

# Get mouse position
def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x = x
        mouse_y = y

def generate_frame(maslab, cap):
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        ''' ================= BEGIN LOGIC ================= '''

        # HSV conversion 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        img = hsv.astype("float32")
        h, s, v = cv2.split(img)
        s = s * maslab.saturation_factor
        s = np.clip(s, 0, 255)
        hsv = cv2.merge([h, s, v])

        ######### WORLD CONSTRUCTION #########

        maslab.world.wheels.update()
        maslab.world.update_cans(hsv)
        maslab.world.get_bounds(hsv)

        ########## ROBOT OPERATIONS ##########

        if maslab.motor_action and len(maslab.world.cans)>0:
            closest = maslab.world.cans[0]
            for can in maslab.world.cans[1:]:
                if can.lowest_point[1]<closest.lowest_point[1]:
                    closest = can
            
            maslab.robot.go_to(closest.lowest_point[0], closest.lowest_point[1])

        ########### VIDEO CREATION ###########

        frame = video.create_video_frame(frame, maslab, (mouse_x, mouse_y))

        ''' ================== END LOGIC ================== '''

        cv2.imshow("Webcam", frame)
        # Process GUI events and allow window to update
        key = cv2.waitKey(1) & 0xFF


def begin_game_loop(self):
    # Open and set up webcam
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.video_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.video_height)
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
    generate_frame(self, cap)

    # Cleanup (at the end)
    cap.release()
    cv2.destroyAllWindows()

MaslabRobot.begin_game_loop = begin_game_loop
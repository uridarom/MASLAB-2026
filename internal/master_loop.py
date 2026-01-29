from internal.MaslabRobot import MaslabRobot
from internal import video
import cv2
import numpy as np
import time
from raven import Raven
from world import Can

# Global variables for mouse position
mouse_x = -1
mouse_y = -1

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
        
        ''' ================= BEGIN GAME LOGIC ================= '''

        # HSV conversion 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        img = hsv.astype("float32")
        h, s, v = cv2.split(img)
        s = s * maslab.saturation_factor
        s = np.clip(s, 0, 255)
        hsv = cv2.merge([h, s, v])

        ######### WORLD CONSTRUCTION #########

        maslab.world.wheels.update()
        maslab.time = time.time()
        maslab.x, maslab.y, maslab.theta = maslab.world.wheels.get_pose_at_time(maslab.time)

        maslab.world.get_bounds(hsv)
        maslab.world.get_green_goal(hsv, frame)
        maslab.world.get_red_goal(hsv, frame)
        maslab.world.get_yellow_goal(hsv, frame)
        maslab.world.update_red_cans(hsv)
        maslab.world.update_green_cans(hsv)
        maslab.world.update_yellow_can(hsv)

        ########## ROBOT OPERATIONS ##########

        cans_list = []
        if maslab.world.red_goal is not None:
            cans_list += maslab.world.red_cans
        if maslab.world.green_goal is not None:
            cans_list += maslab.world.green_cans
        if maslab.world.yellow_goal is not None: 
            cans_list += maslab.world.yellow_cans

        if maslab.motor_action:
            if maslab.robot.depositing:
                if not maslab.robot.is_motor_going(maslab.CHANNEL_3):
                    # Remove current can from list
                    maslab.robot.aligned = False
                    if maslab.robot.active_can.color == Can.CanColor.RED:
                        print("Removed red can")
                        maslab.world.red_cans.remove(maslab.robot.active_can)
                    elif maslab.robot.active_can.color == Can.CanColor.GREEN:
                        maslab.world.green_cans.remove(maslab.roobt.active_can)
                    else:
                        maslab.world.yellow_cans.remove(maslab.robot.active_can)
    
                    maslab.robot.pursuing_can = False
                    maslab.robot.depositing = False
            else:
                if maslab.robot.in_possession_of_can:
                    if not maslab.robot.aligned:
                        # If goal location is known, go to 
                        goal = maslab.robot.active_can.get_goal()
                        maslab.robot.go_to(*goal.nearest_point())
                    else:
                        # Once at goal, turn to face its center
                        maslab.robot.rotating = True
                        maslab.robot.go_to(*goal.get_centroid(), turn_only=True)
                        if not maslab.robot.rotating:
                            maslab.robot.deposit_can()

                else:    
                    if not maslab.robot.pursuing_can:
                        # Select can 
                        closest = None
                        for can in cans_list:
                            if closest is not None:
                                if can.distance_from_robot()<closest.distance_from_robot() and can.confirmed:
                                    closest = can
                            elif can.confirmed:
                                closest = can
                        if closest is not None:
                            print("Replaced can")
                            maslab.robot.pursuing_can = True
                            maslab.robot.active_can = closest
                    elif not maslab.robot.aligned:
                        # Go to can
                        maslab.robot.can_obligated = True
                        maslab.robot.go_to(*maslab.robot.active_can.coords)

                    else:
                        maslab.robot.take_can()
                        maslab.robot.aligned = False
                        maslab.robot.going_to_goal = True

        ########### VIDEO CREATION ###########

        frame = video.create_video_frame(frame, maslab, (mouse_x, mouse_y))

        ''' ================== END GAME LOGIC ================== '''

        cv2.imshow("Webcam", frame)
        # Process GUI events and allow window to update
        key = cv2.waitKey(1) & 0xFF


def begin_game_loop(self):
    # Open and set up webcam
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.video_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.video_height)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) # Enable image compression
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Set to manual exposure mode
    cap.set(cv2.CAP_PROP_EXPOSURE, 250)  # Set exposure time to 5ms
    cap.set(cv2.CAP_PROP_AUTO_WB, 0) # Disable auto white balance
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3500) # Set white balance temperature to 4200K
    cap.set(cv2.CAP_PROP_FPS, 30) # Set frames per second
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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
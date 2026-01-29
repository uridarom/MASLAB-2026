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

def remove_active_can(maslab):
     for can in maslab.robot.active_cans:
        can.confirmed = False
        if can in maslab.world.green_cans:
            maslab.world.green_cans.remove(maslab.robot.active_can)
        elif can in maslab.world.red_cans:
            maslab.world.red_cans.remove(maslab.robot.active_can)
        elif can in maslab.world.yellow_cans:
            maslab.world.yellow_cans.remove(maslab.robot.active_can)

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

        # Last resort: sabatoge
        if maslab.ticks_lost>1000:
            maslab.speed = 100
            maslab.robot.go_to(-50, 450)
        else:
            cans_list = []
            if maslab.world.red_goal is not None:
                cans_list.append(maslab.world.red_cans)
            if maslab.world.green_goal is not None:
                cans_list.append(maslab.world.green_cans)
            if maslab.world.yellow_goal is not None: 
                cans_list.append(maslab.world.yellow_cans)

            if maslab.robot.depositing:
                if not maslab.robot.is_motor_going(maslab.CHANNEL_3):
                    # Remove current can from list
                    maslab.robot.aligned = False
                    remove_active_can(maslab)
                    maslab.robot.can_obligated = False
                    maslab.robot.pursuing_can = False

                    maslab.status = "RESUMING CAN SEARCH"
                    #Look back at cans
                    if not maslab.robot.backing_up and not maslab.robot.backed_up:
                        for channel in (maslab.CHANNEL_1, maslab.CHANNEL_2):
                            maslab.raven.set_motor_max_current(channel, 5) # Set motor current to 5 amps
                            maslab.raven.set_motor_mode(channel, Raven.MotorMode.POSITION) # Set motor mode to POSITION
                            maslab.raven.set_motor_pid(channel, p_gain = 100, i_gain = 0, d_gain = 0, percent = 20) # Set PID values and 20% effort to reduce speed
                        maslab.robot.set_encoder_position(-1600, 1600, relative=True)
                        maslab.robot.backing_up = True
                        maslab.robot.rotating = True
                        maslab.robot.backed_up = False
                        maslab.wait_ticks = 25

                    if maslab.wait_ticks > 0:
                        maslab.wait_ticks -= 1
                    elif maslab.wait_ticks == 0:
                        maslab.robot.backing_up = False
                        maslab.robot.backed_up = True

                    if maslab.robot.backed_up:
                        maslab.robot.go_to(0, 300, turn_only=True)
                        if not maslab.robot.rotating:
                            if maslab.wait_ticks>0:
                                maslab.wait_ticks -= 1
                            if maslab.wait_ticks == 0:
                                maslab.robot.depositing = False
                                maslab.robot.backing_up = False
                                maslab.robot.backed_up = False

            else:
                if maslab.robot.in_possession_of_can:
                    if not maslab.robot.aligned:
                        maslab.status = "GOING TO GOAL"
                        # If goal location is known, go to 
                        goal = maslab.robot.active_can.get_goal()
                        maslab.robot.go_to(*goal.nearest_point())
                    else:
                        # Once at goal, turn to face its center
                        maslab.status = "ALIGNING WITH GOAL"
                        maslab.robot.rotating = True
                        maslab.robot.go_to(*goal.get_centroid(), turn_only=True)
                        if not maslab.robot.rotating:
                            maslab.status = "DEPOSITING CAN"
                            maslab.robot.deposit_can()
                            maslab.robot.in_possession_of_can = False

                else:    
                    if not maslab.robot.pursuing_can:
                        # Select can 
                        closest = None
                        for list in cans_list:
                            for can in list:
                                if closest is not None:
                                    if (can.distance_from_robot()<closest.distance_from_robot() or can.color==Can.CanColor.YELLOW) and can.confirmed and can != maslab.robot.active_can:
                                        closest = can
                                elif can.confirmed:
                                    closest = can
                        if closest is not None:
                            maslab.ticks_lost = 0
                            maslab.robot.pursuing_can = True
                            maslab.robot.active_can = closest
                            maslab.raven.set_motor_speed_factor(maslab.CHANNEL_1, 0)
                            maslab.raven.set_motor_speed_factor(maslab.CHANNEL_2, 0)
                        elif maslab.motor_action:
                            # If can isn't found, spin around
                            maslab.status = "SCANNING FOR CANS"
                            maslab.ticks_lost += 1
                            maslab.robot.turn_factor = 0
                            for channel in (maslab.CHANNEL_1, maslab.CHANNEL_2):
                                maslab.raven.set_motor_mode(channel, Raven.MotorMode.DIRECT)
                                maslab.raven.set_motor_torque_factor(channel, 100)
                            maslab.raven.set_motor_speed_factor(maslab.CHANNEL_1, 8)
                            maslab.raven.set_motor_speed_factor(maslab.CHANNEL_2, 8)

                    elif not maslab.robot.aligned:
                        maslab.status = "GOING TO CAN"
                        # Go to can
                        maslab.robot.can_obligated = True
                        if maslab.motor_action:
                            maslab.robot.go_to(*maslab.robot.active_can.coords)

                    else:
                        maslab.robot.active_cans.append(maslab.robot.active_can)
                        maslab.robot.take_can()
                        maslab.robot.aligned = False
                        if len(maslab.robot.active_cans)>1:
                            maslab.robot.going_to_goal = True
                            maslab.robot.in_possession_of_can = True
                        else:
                            print("Taking 2nd Can")
                            remove_active_can(maslab)
                            maslab.robot.in_possession_of_can = False
                            maslab.robot.pursuing_can = False
        
        print(maslab.robot.in_possession_of_can)

        ########### VIDEO CREATION ###########

        frame = video.create_video_frame(frame, maslab, (mouse_x, mouse_y))

        k = cv2.waitKey(1)
        if k == ord('s') or k == ord('S'):
            maslab.motor_action = True

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
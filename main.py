import sys
import cv2
import numpy as np
from internal.MaslabRobot import MaslabRobot
from internal import master_loop
from world import get_cans
from world import get_bounds
from robot import motor_control

if __name__ == "__main__":
    # Optional argument to disable motors
    motors = True
    test = False
    if len(sys.argv)==2:
        if sys.argv[1].lower() == "viewonly":
            motors = False
            print("ROBOT: Driving Disabled")
        elif sys.argv[1].lower() == "test":
            test = True
            print("Test Mode Enabled")
    
    if not test:
        maslab = MaslabRobot(motor_action=motors)
        maslab.begin_game_loop()
    else:
        ''' ============== BEGIN TEST LOGIC =============='''

        pass

        ''' ============== END TEST LOGIC =============='''


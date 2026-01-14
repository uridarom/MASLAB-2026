import sys
from MaslabRobot import MaslabRobot

if __name__ == "__main__":
    # Optional argument to disable motors
    motors = True
    if len(sys.argv)==2:
        if sys.argv[1].lower() == "viewonly":
            motors = False
            print("ROBOT: Driving Disabled")
    
    robot = MaslabRobot(motor_action=motors)
    robot.begin_tracking()
import sys
from internal.MaslabRobot import MaslabRobot
from internal import master_loop
from world import get_cans
from world import get_bounds
from world import homography
from robot import drive_control
from robot import pickup_control
from robot import motor_control
from raven import Raven

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

        raven = Raven()

        # Deposit can test
        raven.set_motor_mode(Raven.MotorChannel.CH3, Raven.MotorMode.DIRECT) 
        raven.set_motor_torque_factor(Raven.MotorChannel.CH3, 100)
        raven.set_motor_speed_factor(Raven.MotorChannel.CH3, 100, reverse=True)

        while True:
            pass

        ''' ============== END TEST LOGIC =============='''
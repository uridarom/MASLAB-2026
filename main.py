import sys
import cv2
import numpy as np
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
    motors = False
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

        raven.set_motor_mode(Raven.MotorChannel.CH3, Raven.MotorMode.DIRECT) # Set motor mode to DIRECT
        # Speed controlled:
        raven.set_motor_torque_factor(Raven.MotorChannel.CH3, 100) # Let the motor use all the torque to get to speed factor
        raven.set_motor_speed_factor(Raven.MotorChannel.CH3, 100, reverse=True) # Spin at 10% max speed in reverse

        # for channel in (Raven.MotorChannel.CH1, Raven.MotorChannel.CH2):
        #     raven.set_motor_encoder(channel, 0) # Reset encoder
        #     raven.set_motor_max_current(channel, 5) # Set motor current to 5 amps
        #     raven.set_motor_mode(channel, Raven.MotorMode.POSITION) # Set motor mode to POSITION
        # DIAMETER = 9.8425
        # diff = ((30.48*4)/(DIAMETER*np.pi))*3200
        # raven.set_motor_pid(Raven.MotorChannel.CH1, p_gain = 100, i_gain = 0, d_gain = 0, percent = 15) # Set PID values and 20% effort to reduce speed
        # raven.set_motor_target(Raven.MotorChannel.CH1, diff)
        # raven.set_motor_pid(Raven.MotorChannel.CH2, p_gain = 100, i_gain = 0, d_gain = 0, percent = 15) # Set PID values and 20% effort to reduce speed
        # raven.set_motor_target(Raven.MotorChannel.CH2, -diff)

        # for channel in (Raven.MotorChannel.CH1, Raven.MotorChannel.CH2): 
        #     raven.set_motor_mode(channel, Raven.MotorMode.DIRECT) # Set motor mode to DIRECT
        #     raven.set_motor_torque_factor(channel, 100) # Let the motor use all the torque to get to speed factor

        # # Speed controlled:
        # raven.set_motor_speed_factor(Raven.MotorChannel.CH1, 15*0.8) # Spin at 10% max speed in reverse
        # raven.set_motor_speed_factor(Raven.MotorChannel.CH2, 15, reverse=True) # Spin at 10% max speed in reverse

        while True:
            pass

        ''' ============== END TEST LOGIC =============='''
from robot.Robot import Robot
from raven import Raven
import numpy as np

def set_encoder_position(self, right: int, left: int, relative: bool, torque_r=1, torque_l=1):
    """
    Sets both motors to given positions.
    Requires that both motors be set to POSITION mode first.
    
    :param right: Right encoder target
    :param left: Left encoder target
    :param relative: Should motor targets be set relative to current position 
    :param torque_r: Right motor torque
    :param torque_l: Left motor torque
    """
    # Right motor
    target_1 = (self.maslab.raven.get_motor_encoder(self.maslab.CHANNEL_1) if relative else 0)+right
    self.maslab.raven.set_motor_pid(self.maslab.CHANNEL_1, p_gain = 100, i_gain = 0, d_gain = 0, percent = torque_r*self.maslab.speed)
    self.maslab.raven.set_motor_target(self.maslab.CHANNEL_1, target_1)
    # Left motor
    target_2 = (self.maslab.raven.get_motor_encoder(self.maslab.CHANNEL_2) if relative else 0)+left
    self.maslab.raven.set_motor_pid(self.maslab.CHANNEL_2, p_gain = 100, i_gain = 0, d_gain = 0, percent = torque_l*self.maslab.speed*self.maslab.left_offset)
    self.maslab.raven.set_motor_target(self.maslab.CHANNEL_2, target_2)

def go_to(self, X: float | int, Y: float | int, rolling=5, relative=False, turn_only=False):
    """
    Send robot to a real-world position.
    
    :param X: Target x coordinate
    :param Y: Target y coordinate
    :param rolling: How far before the target coordinates should the robot stop
    :param relative: Should the target be relative to the robot's current position
    :param turn_only: Only turn to face the target coordinates (don't drive there)
    """
    # Establish coordinates and angle error
    if not relative:
        dx, dy = self.maslab.world.transform_to_robot(X, Y)
    else:
        dx, dy = (X, Y)
    dy = dy - rolling
    angle_error = np.arctan2(dx, dy)

    # Set motors to direct drive
    for channel in (self.maslab.CHANNEL_1, self.maslab.CHANNEL_2):
        self.maslab.raven.set_motor_mode(channel, Raven.MotorMode.DIRECT)
        self.maslab.raven.set_motor_torque_factor(channel, 100)

    ################ Parameters ################
    ANGLE_DEADBAND = 0.5
    TURN_GAIN = 3.0
    MAX_TURN = 0.15
    BASE_SPEED = self.maslab.speed

    ################ Turn in place if very far off ################
    # If turn only and within 0.1 radians, stop
    if turn_only and self.rotating and abs(angle_error)<0.1:
        self.rotating = False
        self.maslab.raven.set_motor_speed_factor(self.maslab.CHANNEL_1, 0)
        self.maslab.raven.set_motor_speed_factor(self.maslab.CHANNEL_2, 0)
        self.maslab.wait_ticks = 25
        return

    if abs(angle_error) > ANGLE_DEADBAND or turn_only:
        self.turn_factor = 10
        turn = np.clip(angle_error, -MAX_TURN, MAX_TURN)

        right = -turn
        left = turn

        self.maslab.raven.set_motor_speed_factor(self.maslab.CHANNEL_1, abs(right * 100), reverse=right < 0)
        self.maslab.raven.set_motor_speed_factor(self.maslab.CHANNEL_2, abs(left * 100), reverse=left > 0)
        return

    if not turn_only:
        ################ Drive straight forward when aligned ################
        if abs(dy) < 10:
            self.turn_factor = 0
            self.aligned = True
            self.maslab.raven.set_motor_speed_factor(self.maslab.CHANNEL_1, 0)
            self.maslab.raven.set_motor_speed_factor(self.maslab.CHANNEL_2, 0)
            return 40

        ################ Drive forward with steering ################
        turn = TURN_GAIN * angle_error * BASE_SPEED
        turn = np.clip(turn, -BASE_SPEED/2, BASE_SPEED/2)
        self.turn_factor = turn

        left = BASE_SPEED - turn
        right = BASE_SPEED + turn

        self.maslab.raven.set_motor_speed_factor(
            self.maslab.CHANNEL_1, abs(left), reverse=left < 0
        )
        self.maslab.raven.set_motor_speed_factor(
            self.maslab.CHANNEL_2, abs(right), reverse=right > 0
        )
        return

Robot.go_to = go_to
Robot.set_encoder_position = set_encoder_position
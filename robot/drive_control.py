from robot.Robot import Robot
from raven import Raven
import numpy as np

# Sets both motors to given positions
def set_encoder_position(self, right, left, relative, torque_r=1, torque_l=1):
    # Right motor
    self.maslab.raven.set_motor_pid(self.maslab.CHANNEL_1, p_gain = 100, i_gain = 0, d_gain = 0, percent = torque_r*self.maslab.speed)
    self.maslab.raven.set_motor_target(self.maslab.CHANNEL_1, (self.maslab.raven.get_motor_encoder(self.maslab.CHANNEL_1) if relative else 0)+right)
    # Left motor
    self.maslab.raven.set_motor_pid(self.maslab.CHANNEL_2, p_gain = 100, i_gain = 0, d_gain = 0, percent = torque_l*self.maslab.speed*self.maslab.left_offset)
    self.maslab.raven.set_motor_target(self.maslab.CHANNEL_2, (self.maslab.raven.get_motor_encoder(self.maslab.CHANNEL_2) if relative else 0)+left)

# Goes to specific point
def go_to_can(self, X, Y):
    x = X-self.maslab.world.wheels.x
    y = Y-self.maslab.world.wheels.y
    # Determine turning direction and magnitude
    self.turn_factor = -2*np.arctan(x/y) + self.maslab.world.wheels.theta

    # Set up motors for driving
    for channel in (self.maslab.CHANNEL_1, self.maslab.CHANNEL_2):
        self.maslab.raven.set_motor_mode(channel, Raven.MotorMode.DIRECT)
        self.maslab.raven.set_motor_torque_factor(channel, 100)

    # Set motor speeds
    # Stop if object is close enough
    if not self.aligned:
        if y<45:
            self.aligned = True
            for channel in (self.maslab.CHANNEL_1, self.maslab.CHANNEL_2):
                # Set motors to use target position
                self.maslab.raven.set_motor_max_current(channel, 5)
                self.maslab.raven.set_motor_mode(channel, Raven.MotorMode.POSITION)

            # Make the motors spin until robot is right in front of target
            revolutions = y/(self.maslab.WHEEL_DIAMETER*np.pi)*3200
            set_encoder_position(self, revolutions, -revolutions, relative=True)
        
        else:
            self.maslab.ticks_lost = 0
            # Set speeds
            speed_1 = min(100, max(0, self.maslab.speed*(1+self.turn_factor)))
            speed_2 = min(100, max(0, self.maslab.speed*(1-self.turn_factor)*self.maslab.left_offset))

            # Command motors
            self.maslab.raven.set_motor_speed_factor(self.maslab.CHANNEL_1, abs(speed_1), reverse=speed_1<0)
            self.maslab.raven.set_motor_speed_factor(self.maslab.CHANNEL_2, abs(speed_2), reverse=speed_2>0)

Robot.go_to_can = go_to_can
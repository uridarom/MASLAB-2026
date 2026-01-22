from robot.Robot import Robot
from raven import Raven

def run_motor(self, revolutions):
    self.maslab.raven.set_motor_encoder(self.maslab.CHANNEL_3, 0) # Reset encoder
    self.maslab.raven.set_motor_max_current(self.maslab.CHANNEL_3, 5) # Set motor current to 5 amps
    self.maslab.raven.set_motor_mode(self.maslab.CHANNEL_3, Raven.MotorMode.POSITION) # Set motor mode to POSITION
    self.maslab.raven.set_motor_pid(self.maslab.CHANNEL_3, p_gain = 100, i_gain = 0, d_gain = 0, percent = 100) # Set PID values and 20% effort to reduce speed

    # Make the motor spin until 4400 counts (10 rev of wheel motor)
    self.maslab.raven.set_motor_target(self.maslab.CHANNEL_3, 3200*revolutions)

def take_can(self):
    run_motor(self, 5)
    self.pursuing_can = False

def deposit_can(self):
    run_motor(self, -5)

Robot.take_can = take_can
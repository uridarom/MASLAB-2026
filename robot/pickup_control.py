from robot.Robot import Robot
from raven import Raven

def run_motor(self, revolutions, torque=75):
    self.maslab.raven.set_motor_encoder(self.maslab.CHANNEL_3, 0) # Reset encoder
    self.maslab.raven.set_motor_max_current(self.maslab.CHANNEL_3, 5) # Set motor current to 5 amps
    self.maslab.raven.set_motor_mode(self.maslab.CHANNEL_3, Raven.MotorMode.POSITION) # Set motor mode to POSITION
    self.maslab.raven.set_motor_pid(self.maslab.CHANNEL_3, p_gain = 100, i_gain = 0, d_gain = 0, percent = torque) # Set PID values and 20% effort to reduce speed

    # Make the motor spin until 4400 counts (10 rev of wheel motor)
    self.maslab.raven.set_motor_encoder(self.maslab.CHANNEL_3, 0)
    self.maslab.raven.set_motor_target(self.maslab.CHANNEL_3, 3200*revolutions)

def take_can(self):
    self.maslab.status = "COLLECTING CAN"
    self.in_possession_of_can = True
    run_motor(self, 5)

def deposit_can(self):
    self.maslab.status = "DEPOSITING CAN"
    self.in_possession_of_can = False
    self.depositing = True

    for channel in (self.maslab.CHANNEL_1, self.maslab.CHANNEL_2):
        self.maslab.raven.set_motor_encoder(channel, 0) # Reset encoder
        self.maslab.raven.set_motor_max_current(channel, 5) # Set motor current to 5 amps
        self.maslab.raven.set_motor_mode(channel, Raven.MotorMode.POSITION) # Set motor mode to POSITION
        self.maslab.raven.set_motor_pid(channel, p_gain = 100, i_gain = 0, d_gain = 0, percent = 20) # Set PID values and 20% effort to reduce speed

    self.set_encoder_position(-3200, 3200, relative=True)
    run_motor(self, -3, torque = 35)

Robot.take_can = take_can
Robot.deposit_can = deposit_can
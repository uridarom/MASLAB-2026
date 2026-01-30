from robot.Robot import Robot
from raven import Raven

def run_motor(self, revolutions, torque=75):
    """
    Run the intake/outtake motor.
    
    :param revolutions: Number of revolutions for motor to spin
    :param torque: Torque (effective speed) for motor to use
    """
    # Reset motor to position
    self.maslab.raven.set_motor_encoder(self.maslab.CHANNEL_3, 0) 
    self.maslab.raven.set_motor_max_current(self.maslab.CHANNEL_3, 5)
    self.maslab.raven.set_motor_mode(self.maslab.CHANNEL_3, Raven.MotorMode.POSITION) # 
    self.maslab.raven.set_motor_pid(self.maslab.CHANNEL_3, p_gain = 100, i_gain = 0, d_gain = 0, percent = torque)

    self.maslab.raven.set_motor_encoder(self.maslab.CHANNEL_3, 0)
    self.maslab.raven.set_motor_target(self.maslab.CHANNEL_3, 3200*revolutions)

def take_can(self):
    """Intake can"""
    self.maslab.status = "COLLECTING CAN"
    run_motor(self, 5)

def deposit_can(self):
    """Deposit can"""
    self.maslab.status = "DEPOSITING CAN"
    self.depositing = True

    # Make robot drive forwards while depositing to knock over can
    for channel in (self.maslab.CHANNEL_1, self.maslab.CHANNEL_2):
        self.maslab.raven.set_motor_max_current(channel, 5) 
        self.maslab.raven.set_motor_mode(channel, Raven.MotorMode.POSITION)
        self.maslab.raven.set_motor_pid(channel, p_gain = 100, i_gain = 0, d_gain = 0, percent = 20)
    self.set_encoder_position(1600, -1600, relative=True)

    run_motor(self, -3, torque = 100)

Robot.take_can = take_can
Robot.deposit_can = deposit_can
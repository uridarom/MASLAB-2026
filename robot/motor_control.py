from robot.Robot import Robot

def is_motor_going(self, channel):
    return abs(abs(self.maslab.raven.get_motor_encoder(channel))-abs(self.maslab.raven.get_motor_target(channel)))>1000

Robot.is_motor_going = is_motor_going
from robot.Robot import Robot

def is_motor_going(self, channel) -> bool:
    """
    Returns boolean that is True if the given motor actively turning
    and False if it is not.
    
    :param channel: Target channel
    """
    return abs(abs(self.maslab.raven.get_motor_encoder(channel))-abs(self.maslab.raven.get_motor_target(channel)))>1000

Robot.is_motor_going = is_motor_going
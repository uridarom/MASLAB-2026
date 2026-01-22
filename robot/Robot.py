from raven import Raven

class Robot:

    def __init__(self, maslab):
        self.maslab = maslab
        self.aligned = False
        self.turn_factor = 0
        self.pursuing_can = False

        for channel in (self.maslab.CHANNEL_1, self.maslab.CHANNEL_2):
            # Reset motors to use target position
            self.maslab.raven.set_motor_encoder(channel, 0)
            self.maslab.raven.set_motor_max_current(channel, 5)
            self.maslab.raven.set_motor_mode(channel, Raven.MotorMode.POSITION)
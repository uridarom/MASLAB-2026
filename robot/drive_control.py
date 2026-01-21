from robot.Robot import Robot
from raven import Raven

# Goes to specific point
def go_to_can(self, x, y):
    distance = self.maslab.video_height-y
    # Determine turning direction and magnitude
    self.turn_factor = ((1-float(x)/(self.maslab.video_width/2))*distance/self.maslab.video_height)*(15/self.maslab.speed)

    # Set up motors for driving
    for channel in (self.maslab.CHANNEL_1, self.maslab.CHANNEL_2):
        self.maslab.raven.set_motor_mode(channel, Raven.MotorMode.DIRECT)
        self.maslab.raven.set_motor_torque_factor(channel, 100)

    # Set motor speeds
    # Stop if object is close enough
    if not self.aligned:
        if abs(self.turn_factor) < self.maslab.slowdown_tolerence and distance<=self.maslab.slowdown_tolerence:
            self.aligned = True
            for channel in (self.maslab.CHANNEL_1, self.maslab.CHANNEL_2):
                # Reset motors to use target position
                self.maslab.raven.set_motor_encoder(channel, 0)
                self.maslab.raven.set_motor_max_current(channel, 5)
                self.maslab.raven.set_motor_mode(channel, Raven.MotorMode.POSITION)

            # Make the motor spin until it is right in front of target

            # Right motor
            self.maslab.raven.set_motor_pid(self.maslab.CHANNEL_1, p_gain = 100, i_gain = 0, d_gain = 0, percent = self.maslab.speed)
            self.maslab.raven.set_motor_target(self.maslab.CHANNEL_1, self.maslab.raven.get_motor_encoder(self.maslab.CHANNEL_1)+440*self.maslab.roll_from_stop)
            # Left motor
            self.maslab.raven.set_motor_pid(self.maslab.CHANNEL_2, p_gain = 100, i_gain = 0, d_gain = 0, percent = self.maslab.speed*self.maslab.left_offset)
            self.maslab.raven.set_motor_target(self.maslab.CHANNEL_2, self.maslab.raven.get_motor_encoder(self.maslab.CHANNEL_2)-440*self.maslab.roll_from_stop)
            
            self.target_drive_encoder_1 = self.maslab.raven.get_motor_encoder(self.maslab.CHANNEL_1)+440*self.maslab.roll_from_stop

        else:
            # Spin in circles if no object was found
            lost = False
            if x == 640 and y == 0 and self.maslab.idle_spinning:
                if self.maslab.ticks_lost>10:
                    lost = True
                    speed_1 = 10
                    speed_2 = -10
                else:
                    self.maslab.ticks_lost += 1
            if not lost:
                self.maslab.ticks_lost = 0
                # Set speeds
                speed_1 = min(100, max(0, self.maslab.speed*(1+self.turn_factor)))
                speed_2 = min(100, max(0, self.maslab.speed*(1-self.turn_factor)*self.maslab.left_offset))

            # Command motors
            self.maslab.raven.set_motor_speed_factor(self.maslab.CHANNEL_1, abs(speed_1), reverse=speed_1<0)
            self.maslab.raven.set_motor_speed_factor(self.maslab.CHANNEL_2, abs(speed_2), reverse=speed_2>0)

Robot.go_to_can = go_to_can
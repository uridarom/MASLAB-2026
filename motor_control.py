import curses
from raven import Raven

CHANNEL_1 = Raven.MotorChannel.CH1
CHANNEL_2 = Raven.MotorChannel.CH2

raven = Raven()

# Set up motors
for ch in (CHANNEL_1, CHANNEL_2):
    raven.set_motor_mode(ch, Raven.MotorMode.DIRECT)
    raven.set_motor_torque_factor(ch, 100)

# Goes to specific point
def go_to(x, y):
    Y = y/720
    # Make speed faster when object is further
    speed = 50*Y
    # Value that determines turning direction
    turn_factor = 1-float(x)/640
    raven.set_motor_speed_factor(CHANNEL_2, min(100, max(0, 25*(1+2*turn_factor))), reverse=False)
    raven.set_motor_speed_factor(CHANNEL_1, min(100, max(0, 25*(1-2*turn_factor))), reverse=True)
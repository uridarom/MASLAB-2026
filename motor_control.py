import curses
from constants import WIDTH, HEIGHT, SPEED, SLOWDOWN_TOLERENCE, SLOWDOWN_TRIGGER, CHANNEL_1, CHANNEL_2
from raven import Raven

raven = Raven()

# Set up motors
for channel in (CHANNEL_1, CHANNEL_2):
    raven.set_motor_mode(channel, Raven.MotorMode.DIRECT)
    raven.set_motor_torque_factor(channel, 100)

# Goes to specific point
def go_to(x, y):
    distance = WIDTH-y
    # Determine turning direction and magnitude
    turn_factor = (1-float(x)/(HEIGHT/2))*distance/WIDTH

    # Set motor speeds
    # Stop if object is close enough
    if abs(turn_factor) < SLOWDOWN_TOLERENCE and distance<=SLOWDOWN_TRIGGER:
        speed_1 = 0
        speed_2 = 0
    else:
        speed_1 = min(100, max(0, SPEED*(1-2*turn_factor)))
        speed_2 = min(100, max(0, SPEED*(1+2*turn_factor)))

    # Command motors
    raven.set_motor_speed_factor(CHANNEL_1, speed_1, reverse=True)
    raven.set_motor_speed_factor(CHANNEL_2, speed_2, reverse=False)

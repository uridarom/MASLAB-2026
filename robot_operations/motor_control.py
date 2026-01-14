import curses
from raven import Raven

raven = Raven()

# Motor channels
CHANNEL_1 = Raven.MotorChannel.CH1
CHANNEL_2 = Raven.MotorChannel.CH2

# Set up motors
for channel in (CHANNEL_1, CHANNEL_2):
    raven.set_motor_mode(channel, Raven.MotorMode.DIRECT)
    raven.set_motor_torque_factor(channel, 100)

# Goes to specific point
def go_to(x, y, robot):
    distance = robot.video_height-y
    # Determine turning direction and magnitude
    turn_factor = (1-float(x)/(robot.video_width/2))*distance/robot.video_height

    # Set motor speeds
    # Stop if object is close enough
    if abs(turn_factor) < robot.slowdown_tolerence and distance<=robot.slowdown_tolerence:
        for channel in (CHANNEL_1, CHANNEL_2):
            # Reset encoder
            raven.set_motor_encoder(channel, 0)
            # Set motor to 5 amps
            raven.set_motor_max_current(channel, 5)
            # Use position
            raven.set_motor_mode(channel, Raven.MotorMode.POSITION)
            # Set PID values, 20% speed
            raven.set_motor_pid(channel, p_gain = 100, i_gain = 0, d_gain = 0, percent = 20)

        # Make the motor spin until it is right in front of target
        raven.set_motor_target(CHANNEL_1, -440*robot.roll_from_stop)
        raven.set_motor_target(CHANNEL_2, 440*robot.roll_from_stop)
    else:
        # Set speeds
        speed_1 = min(100, max(0, robot.speed*(1-2*turn_factor)))
        speed_2 = min(100, max(0, robot.speed*(1+2*turn_factor)*robot.right_offset))

        # Command motors
        raven.set_motor_speed_factor(CHANNEL_1, speed_1, reverse=True)
        raven.set_motor_speed_factor(CHANNEL_2, speed_2, reverse=False)

    return turn_factor, distance

from raven import Raven

CHANNEL_1 = Raven.MotorChannel.CH1
CHANNEL_2 = Raven.MotorChannel.CH2

raven_board = Raven()

# Set both motors to DIRECT mode
raven_board.set_motor_mode(CHANNEL_1, Raven.MotorMode.DIRECT)
raven_board.set_motor_mode(CHANNEL_2, Raven.MotorMode.DIRECT)

# Motor 1
raven_board.set_motor_torque_factor(CHANNEL_1, 100)
raven_board.set_motor_speed_factor(CHANNEL_1, 30)

# Motor 2
raven_board.set_motor_torque_factor(CHANNEL_2, 100)
raven_board.set_motor_speed_factor(CHANNEL_2, 30, reverse=True)
while True:
    pass
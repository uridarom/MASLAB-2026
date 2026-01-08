import curses
from raven import Raven

CHANNEL_1 = Raven.MotorChannel.CH1
CHANNEL_2 = Raven.MotorChannel.CH2

raven = Raven()

for ch in (CHANNEL_1, CHANNEL_2):
    raven.set_motor_mode(ch, Raven.MotorMode.DIRECT)
    raven.set_motor_torque_factor(ch, 100)

def main(stdscr):
    stdscr.nodelay(True)
    stdscr.clear()

    while True:
        key = stdscr.getch()

        if key == ord('w'):
            raven.set_motor_speed_factor(CHANNEL_1, 30)
            raven.set_motor_speed_factor(CHANNEL_2, 30, reverse=True)
        elif key == ord('s'):
            raven.set_motor_speed_factor(CHANNEL_1, 30, reverse=True)
            raven.set_motor_speed_factor(CHANNEL_2, 30)
        elif key == ord('a'):
            raven.set_motor_speed_factor(CHANNEL_1, 20)
            raven.set_motor_speed_factor(CHANNEL_2, 20)
        elif key == ord('d'):
            raven.set_motor_speed_factor(CHANNEL_1, 20, reverse=True)
            raven.set_motor_speed_factor(CHANNEL_2, 20, reverse=True)
        elif key == ord('q'):
            raven.set_motor_speed_factor(CHANNEL_1, 0)
            raven.set_motor_speed_factor(CHANNEL_2, 0)
            break

curses.wrapper(main)

from raven import Raven

''' ------------ can_detection.py ------------ '''
# Image dimensions
WIDTH = 1280
HEIGHT = 720
# Minimum rectangular area for object to be flagged
AREA_MIN = 5000
# How far from the ideal (1.50) aspect ratio may an object be
RATIO_TOLERENCE = 0.2

''' ------------ motor_control.py ------------ '''
# Universal speed reference
SPEED = 25
# How straight-on target must be before slowing down
SLOWDOWN_TOLERENCE = 0.01
# How close target must be before slowing down
SLOWDOWN_TRIGGER = 0
# Motor channels
CHANNEL_1 = Raven.MotorChannel.CH1
CHANNEL_2 = Raven.MotorChannel.CH2
from raven import Raven
from robot_operations.can_detection import begin_tracking

class MaslabRobot:

    def __init__(
        self, 
        # Image dimensions
        video_width = 1280, 
        video_height = 720, 
        # Minimum rectangular area for object to be flagged
        area_min = 2000, 
        # How far from the ideal (1.50) aspect ratio may an object be
        ratio_tolerence = 0.2, 
        # Should robot drive to can
        motor_action = True, 
        # Hue, Saturation, Brightness tolerences for color detection
        color_tolerences = (10, 230, 80), 

        # Universal speed reference
        speed = 25, 
        # How straight-on target must be before slowing down
        slowdown_tolerence = 0.01, 
        # How close target must be before slowing down
        slowdown_trigger = 10, 
        # Offset to counter left-turning tendency
        right_offset = 0.93, 
        # How many revolutions of the wheel robot should continue after reaching distance threshold
        roll_from_stop = 2.5,
        # How much to increase image saturation
        saturation_factor = 3,
        # How far past a boundry can the robot target (in pixels)
        boundry_padding = 0
    ):
        ''' ------------ can_detection.py ------------ '''
        self.video_width = video_width
        self.video_height = video_height
        self.area_min = area_min
        self.ratio_tolerence = ratio_tolerence
        self.motor_action = motor_action
        self.color_tolerences = color_tolerences
        self.saturation_factor = saturation_factor
        self.boundry_padding = boundry_padding

        ''' ------------ motor_control.py ------------ '''
        self.speed = speed
        self.slowdown_tolerence = slowdown_tolerence
        self.slowdown_trigger = slowdown_trigger
        self.right_offset = right_offset
        self.roll_from_stop = roll_from_stop

    # Track cans
    def begin_tracking(self):
        print("ROBOT: Beginning can tracking")
        begin_tracking(self)
from raven import Raven
from world.World import World
from robot.Robot import Robot

class MaslabRobot:

    def __init__(
        self, 
        # Image dimensions
        video_width = 1280, 
        video_height = 720, 
        # Minimum rectangular area for object to be flagged
        area_min = 1000, 
        # How far from the ideal (1.50) aspect ratio may an object be
        ratio_tolerence = 0.2, 
        # Should robot drive to can
        motor_action = True, 

        # Universal speed reference
        speed = 20, 
        # How straight-on target must be before slowing down
        slowdown_tolerence = 0.01, 
        # How close target must be before slowing down
        slowdown_trigger = 15, 
        # Offset to counter left-turning tendency
        left_offset = 1.0, 
        # How many revolutions of the wheel robot should continue after reaching distance threshold
        roll_from_stop = 3,
        # How much to increase image saturation
        saturation_factor = 3,
        # How far past a boundry can the robot target (in pixels)
        boundry_padding = 0,
        # Should the robot spin in circles if no can is found
        idle_spinning = False,
        # How close a newly detected can has to be to a previously logged one to assume its the same can (in cm)
        can_proximity_tolerence = 5,
        # How large the robot should be in the map (in pixels)
        robot_size = 40,
        # How zoomed in the map should be 
        map_scale = 2,
    ):
        self.video_width = video_width
        self.video_height = video_height
        self.area_min = area_min
        self.ratio_tolerence = ratio_tolerence
        self.motor_action = motor_action
        self.saturation_factor = saturation_factor
        self.boundry_padding = boundry_padding
        self.speed = speed
        self.slowdown_tolerence = slowdown_tolerence
        self.slowdown_trigger = slowdown_trigger
        self.left_offset = left_offset
        self.roll_from_stop = roll_from_stop
        self.idle_spinning = idle_spinning
        self.can_proximity_tolerence = can_proximity_tolerence
        self.robot_size = robot_size
        self.map_scale = map_scale

        self.CHANNEL_1 = Raven.MotorChannel.CH1
        self.CHANNEL_2 = Raven.MotorChannel.CH2
        self.CHANNEL_3 = Raven.MotorChannel.CH3
        self.WHEEL_DIAMETER = 9.8425
        self.WHEEL_WIDTH = 2.000
        self.TRACK_WIDTH = 23.5
        self.CPR = 3200
        self.wait_ticks = 0

        self.status = "MAPPING"

        self.raven = Raven()
        self.world = World(self)
        self.robot = Robot(self)
    
    def print(text):
        print("DEBUG: "+text)
        
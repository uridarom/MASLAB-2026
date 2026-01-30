from world.WheelOdometry import WheelOdometry
from world.Bounds import Bounds
import numpy as np

class World:
    """
    World object that contains environment info and related methods
    """
    def __init__(self, maslab):
        self.maslab = maslab
        self.wheels = WheelOdometry(
            self.maslab,
            wheel_diameter = maslab.WHEEL_DIAMETER, 
            track_width = maslab.TRACK_WIDTH, 
            cpr = maslab.CPR)
        
        self.cans = []
        self.red_cans = []
        self.yellow_cans = []
        self.green_cans = []
        self.border = ((0, 0), (0, 0, 0, 0))
        self.bounds = Bounds(self)

        self.red_goal = None
        self.green_goal = None
        self.yellow_goal = None
        self.taken_yellow = False
from world.WheelOdometry import WheelOdometry

class World:

    def __init__(self, maslab):
        self.maslab = maslab
        self.wheels = WheelOdometry(
            self.maslab,
            wheel_diameter = maslab.WHEEL_DIAMETER, 
            track_width = maslab.TRACK_WIDTH, 
            cpr = maslab.CPR)
        
        self.cans = []
        self.border = ((0, 0), (0, 0, 0, 0))
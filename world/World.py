from world.WheelOdometry import WheelOdometry

class World:

    def __init__(self, maslab):
        self.maslab = maslab
        self.wheels = WheelOdometry(
            wheel_diameter = 9.8, 
            track_width = 20.4, 
            cpr = 1000)
        
        self.cans = []
        self.border = ((0, 0), (0, 0, 0, 0))
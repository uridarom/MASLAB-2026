class Robot:

    def __init__(self, maslab):
        self.maslab = maslab
        self.aligned = False
        self.aligned_ticks = 0
        self.turn_factor = 0
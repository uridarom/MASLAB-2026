from raven import Raven

class Robot:

    def __init__(self, maslab):
        self.maslab = maslab
        self.aligned = False
        self.turn_factor = 0
        self.pursuing_can = False
        self.going_to_goal = False
        self.target_position = None
        self.turned_around = False
        self.active_can = None
        self.active_cans = []
        self.in_possession_of_can = False
        self.rotating = False
        self.depositing = False
        self.locked_can = False
        self.can_obligated = False
        self.backing_up = False
        self.backed_up = False
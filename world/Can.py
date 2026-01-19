from enum import Enum

class CanColor(Enum):
    RED = 0
    GREEN = 1

class Can:
    def __init__(self, rect, color, in_bounds=True, in_goal=False):
        self.rect = rect
        self.color = color
        self.in_bounds = in_bounds
        self.in_goal = in_goal
        self.lowest_point = (int(self.rect[0] + self.rect[2]/2), self.rect[1] + self.rect[3])
    
    def __repr__(self):
        return f"Rect: {self.rect}\nColor: {"RED" if self.color==0 else "GREEN"}\nIn bounds: {self.in_bounds}\nIn goal: {self.in_goal}"

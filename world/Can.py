from enum import Enum
from world import homography

class CanColor(Enum):
    RED = 0
    GREEN = 1

class Can:
    def __init__(self, world, rect, color, in_bounds=True, in_goal=False):
        self.rect = rect
        self.color = color
        self.in_bounds = in_bounds
        self.in_goal = in_goal
        self.lowest_point = (int(self.rect[0] + self.rect[2]/2), self.rect[1] + self.rect[3])
        self.in_view = False
        self.replaced = 0
        self.confirmed = False
        self.world = world

        x, y = homography.transform_uv_to_xy(self.lowest_point[0], self.lowest_point[1])
        self.coords = (self.world.wheels.x + x, self.world.wheels.y + y)
    
    def update(self, rect):
        self.in_view = True
        self.rect = rect
        self.lowest_point = (int(self.rect[0] + self.rect[2]/2), self.rect[1] + self.rect[3])
        x, y = homography.transform_uv_to_xy(self.lowest_point[0], self.lowest_point[1])
        self.coords = (self.world.wheels.x + x, self.world.wheels.y + y)

        if self.replaced == 5:
            self.confirmed = True
        else:
            self.replaced += 1
    
    def __repr__(self):
        return f"Rect: {self.rect}\nColor: {"RED" if self.color==0 else "GREEN"}\nIn bounds: {self.in_bounds}\nIn goal: {self.in_goal}"

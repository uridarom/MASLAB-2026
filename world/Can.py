from enum import Enum
import numpy as np
from world.Goal import Goal

class CanColor(Enum):
    RED = 0
    GREEN = 1
    YELLOW = 2

class Can:
    """
    Can object that represents an individual can in real-world space.
    """
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
        self.ticks_lost = 0

        self.coords = tuple(self.world.transform_uv_to_xy(*self.lowest_point))
        self.relative_coords = tuple(self.world.transform_uv_to_xy_relative(*self.lowest_point))
        
    
    def update(self, rect: list | tuple):
        """
        Update can with new location, where rect is the bounding
        rectangle of a can in image coordinates. After 5 updates, the
        can is considered confirmed and can be used in-game.
        
        :param rect: List of bounding rectangle points in the format (x, y, w, h)
        """
        if abs(self.world.maslab.robot.turn_factor)<10:
            self.in_view = True
            self.rect = rect
            self.lowest_point = (int(self.rect[0] + self.rect[2]/2), self.rect[1] + self.rect[3])
            new_coords = tuple(self.world.transform_uv_to_xy(*self.lowest_point))
            # Low pass can coords for stability
            self.coords = tuple(0.1 * np.array(new_coords) + (1 - 0.1) * np.array(self.coords))
            self.relative_coords = tuple(self.world.transform_uv_to_xy_relative(*self.lowest_point))

            # After 5 replacements, confirm can
            if self.replaced > 5:
                self.confirmed = True
            self.replaced += 1

            # De-confirm can if out of bounds
            self.in_bounds = self.world.bounds.is_point_in_bounds(*self.coords)
            if not self.in_bounds:
                self.confirmed = False
        
    def distance_from_robot(self) -> float:
        """
        Calculates the basic trigonometric distance from the can to the robot.
        """
        x = self.coords[0] - self.world.wheels.x
        y = self.coords[1] - self.world.wheels.y
        return np.sqrt(x**2 + y**2)

    def get_color(self) -> tuple:
        """
        Returns color of can in (B, G, R) format
        """
        if self.world.maslab.robot.active_can == self:
            color = (255, 150, 0)
        elif self.color == CanColor.RED:
            color = (0, 0, 255)
        elif self.color == CanColor.GREEN:
            color = (0, 255, 0)
        else:
            color = (0, 255, 255)
        return color

    def get_goal(self) -> Goal:
        """
        Returns appropriate goal object
        """
        if self.color == CanColor.RED:
            return self.world.red_goal
        elif self.color == CanColor.GREEN:
            return self.world.green_goal
        else:
            return self.world.yellow_goal
    
    def get_list(self) -> list:
        """
        Rwturns appropriate list of cans (of the same color)
        """
        if self.color == CanColor.RED:
            return self.world.red_cans
        elif self.color == CanColor.GREEN:
            return self.world.green_cans
        else:
            return self.world.yellow_cans
    
    def __repr__(self):
        return f"Rect: {self.rect}\nColor: {"RED" if self.color==0 else "GREEN"}\nIn bounds: {self.in_bounds}\nIn goal: {self.in_goal}"

import numpy as np
import math

def get_slope(points: list | tuple) -> float | None:
    """
    Get slope of a line.
    
    :param points: List of points. Interpreted as [x1, y1, x2, y2].
    """
    if points[1][0]-points[0][0] != 0:
        return (points[1][1]-points[0][1])/(points[1][0]-points[0][0])
    return None

class Bounds:
    def __init__(self, world):
        self.world = world
        self.in_view = False
        self.lines = []
    
    def update(self, x1: float | int, y1: float | int, x2: float | int, y2: float | int):
        """
        Add or update a boundary line from given real-world segment.
        The provided endpoints are interpreted as real-world coordinates. 
        If slope of segment is similar enough to existing line, it is updated.
        Otherwise a new line is created.

        :param x1, y1: The first segment endpoint
        :param x2, y2: The second segment endpoint
        """
        p1 = (x1, y1)
        p2 = (x2, y2)
        # If similar bound line present, update it; otherwise create new
        if len(self.lines)<1:
            self.lines.append(BoundLine(self.world, p1, p2))
        else:
            added = False
            for line in self.lines:
                if line.update(p1, p2):
                    added = True
                    break
            if not added:
                self.lines.append(BoundLine(self.world, p1, p2))
    
    def signed_distance(self, x1: float, y1: float, x2: float, y2: float, x: float, y: float) -> float | None:
        """
        Compute the signed perpendicular distance from point (x, y)
        to the directed line passing through (x1, y1) -> (x2, y2).

        The sign of the result indicates which side of the line the point
        lies on relative to the direction from (x1, y1) to (x2, y2).

        :param x1, y1: first endpoint of the line segment
        :param x2, y2: second endpoint of the line segment
        :param x, y: point to compute the signed distance for

        Returns
        - float: signed perpendicular distance (same units as inputs)
        - None: if the line endpoints are identical (zero-length line)
        """
        num = (x2 - x1)*(y - y1) - (y2 - y1)*(x - x1)
        den = math.hypot(x2 - x1, y2 - y1)
        if den == 0:
            return None
        return num / den

    def crosses_line(self, x1: float, y1: float, x2: float, y2: float, a: float, b: float, tol=5) -> bool:
        """
        Determine whether the robot must cross the line from (x1, y1) to (x2, y2)
        in order to reach a target (a, b), with a tolerence given by tol.

        :param x1, y1, x2, y2: Endpoints of the boundary line
        :param a, b: Coordinates of the target point
        :param tol: Ignore band around the line; if either
          point lies within `tol` of the line, the function returns False

        Returns
        - bool: True if the robot and target are on opposite sides of the
          line; False otherwise.
        """
        d_robot = self.signed_distance(x1, y1, x2, y2, 0, 0)
        d_target = self.signed_distance(x1, y1, x2, y2, a, b)

        if d_robot is not None and d_target is not None:
            # if either is within tolerance band, ignore
            if abs(d_robot) < tol or abs(d_target) < tol:
                return False

            # crossing only if opposite sides
            return d_robot * d_target < 0
        else:
            return False

    def is_point_in_bounds(self, x: float, y: float) -> bool:
        """
        Given a real-world coordinate point (x, y), checks if the robot must cross 
        any known boundry line to get to that point.
        
        :param x, y: Target point
        """
        for bound_line in self.lines:
            if self.crosses_line(*bound_line.p1, *bound_line.p2, x, y) or y>360:
                return False
        return True

class BoundLine:
    def __init__(self, world, p1, p2):
        self.world = world
        self.p1 = self.world.transform_uv_to_xy(*p1, integer=True)
        self.p2 = self.world.transform_uv_to_xy(*p2, integer=True)
        self.slope = get_slope((self.p1, self.p2))
        self.confirmed = False
        self.updated = 0
    
    def update(self, p1: tuple | list, p2: tuple | list) -> bool:
        """
        Updates line with new information if new line is within slope tolerence.
        
        :param p1: The first endpoint of line
        :param p2: The second endpoint of line

        Returns:
        - bool: True if new line was accepted, False otherwise.
        """
        real_1 = self.world.transform_uv_to_xy(*p1, integer=True)
        real_2 = self.world.transform_uv_to_xy(*p2, integer=True)
        slope = get_slope((real_1, real_2))

        if real_2[1]>360:
            return False
        
        # If slope is similar, update, otherwise ignore
        if slope is not None and self.slope is not None:
            if abs(slope-self.slope)<0.5:
                self.p1 = ((real_1[0]+self.p1[0])/2, (real_1[1]+self.p1[1])/2)
                self.p2 = ((real_2[0]+self.p2[0])/2, (real_2[1]+self.p2[1])/2)
                self.slope = get_slope((self.p1, self.p2))
                self.updated += 1
                if self.updated >= 20:
                    self.confirmed = True

                    # Get unit vector
                    unit = np.array(self.p2) - np.array(self.p1)
                    unit = unit / np.linalg.norm(unit) 
                    self.unit = unit

                return True
        return False
      

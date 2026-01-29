import numpy as np

def get_slope(points):
    if points[1][0]-points[0][0] != 0:
        return (points[1][1]-points[0][1])/(points[1][0]-points[0][0])
    return None

class Bounds:
    def __init__(self, world):
        self.world = world
        self.in_view = False
        self.lines = []
    
    def update(self, x1, y1, x2, y2):
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
    
    def crosses_line(self, x1, y1, x2, y2, a, b):
        def side(x, y):
            return (x2 - x1)*(y - y1) - (y2 - y1)*(x - x1)

        s0 = side(0, 0)
        sq = side(a, b)

        if s0 == 0 or sq == 0:
            return "on the line"
        elif s0 * sq < 0:
            return True
        else:
            return False

    def is_point_in_bounds(self, x, y):
        for bound_line in self.lines:
            if self.crosses_line(*bound_line.p1, *bound_line.p2, x, y):
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
    
    def update(self, p1, p2):
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
      

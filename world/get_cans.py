import cv2
import numpy as np
from world import Can
from world.World import World

def rect_to_vertices(x, y, w, h):
    return np.array([
        [x,     y],
        [x+w,   y],
        [x+w,   y+h],
        [x,     y+h]
    ])

def project(poly, axis):
    dots = [np.dot(p, axis) for p in poly]
    return min(dots), max(dots)

def overlap_1d(a, b):
    return not (a[1] < b[0] or b[1] < a[0])

def axes(poly):
    axes = []
    for i in range(len(poly)):
        p1 = poly[i]
        p2 = poly[(i+1) % len(poly)]
        edge = p2 - p1
        normal = np.array([-edge[1], edge[0]])
        normal = normal / np.linalg.norm(normal)
        axes.append(normal)
    return axes

def polygons_overlap(poly1, poly2):
    for axis in axes(poly1) + axes(poly2):
        p1 = project(poly1, axis)
        p2 = project(poly2, axis)
        if not overlap_1d(p1, p2):
            return False
    return True

# Creates video output and commands motors
def update_cans(self, mask, can_list, color):
    # Clean mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find individual objects
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    # Create list of potential objects
    rectangles = []
    for cnt in contours:
        # Rectangular bounding box
        x, y, w, h = cv2.boundingRect(cnt)
        # Get area of contour (not of rectangle)
        area = cv2.contourArea(cnt)

        if w*h < self.maslab.area_min:
            continue

        # Ignore cans if cut off by FOV
        if y < 2 or (y+h) > self.maslab.video_height-2:
            continue
        if h / w < 1:
            continue

        # How well contour fills the bounding box
        rect_area = w * h
        extent = area / rect_area
        if extent < 0.6:
            continue
        
        rectangles.append((x, y, w, h))
    
    for can in can_list:
        can.in_view = False

    # Check for overlap, remove smaller boxes if present
    for rect in rectangles:
        keep = True
        for ref_rect in rectangles:
            if polygons_overlap(rect_to_vertices(*rect), rect_to_vertices(*ref_rect)):
                if rect[2]*rect[3] < ref_rect[2]*ref_rect[3]:
                    keep = False
                    break
        if keep:
            
            # Make sure a goal isn't mistaken for a can
            for goal in (self.red_goal, self.green_goal, self.yellow_goal):
                if goal is not None and goal.in_view:
                    if polygons_overlap(rect_to_vertices(*rect), goal.quad):
                        print("Canccled")
                        return
                    
            can = Can.Can(self, rect, color)
            replaced = False
            best_dist = 2**31
            best_index = -1
            for i, old_can in enumerate(can_list):
                dist = np.sqrt((can.coords[0]-old_can.coords[0])**2 + (can.coords[1]-old_can.coords[1])**2)
                if dist<best_dist:
                    best_dist = dist
                    best_index = i
            tolerence = self.maslab.can_proximity_tolerence
            if self.maslab.robot.can_obligated:
                tolerence *= 10
            if best_dist<tolerence:
                can_list[best_index].update(rect)
                replaced = True
            if not replaced:
                if not self.maslab.robot.turn_factor>0:
                    can_list.append(can)
    
    return can_list
                
def update_red_cans(self, hsv):
    # Hue / Saturation / Brightness ranges
    lower_red1 = np.array([0, 190, 80])
    upper_red1 = np.array([15, 255, 255])

    lower_red2 = np.array([165, 190, 80])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    self.red_cans = update_cans(self, mask, self.red_cans, Can.CanColor.RED)

def update_yellow_can(self, hsv):
    # Hue / Saturation / Brightness ranges
    lower_yellow = np.array([20, 190, 150])
    upper_yellow = np.array([35, 255, 255])

    mask_yellow = (cv2.inRange(hsv, lower_yellow, upper_yellow))

    self.yellow_cans = update_cans(self, mask_yellow, self.yellow_cans, Can.CanColor.YELLOW)

def update_green_cans(self, hsv):
    # Hue / Saturation / Brightness ranges
    lower_green = np.array([40, 90, 60])
    upper_green = np.array([70, 255, 255])

    mask_green = (cv2.inRange(hsv, lower_green, upper_green))

    self.green_cans = update_cans(self, mask_green, self.green_cans, Can.CanColor.GREEN)


World.update_cans = update_cans                
World.update_red_cans = update_red_cans
World.update_green_cans = update_green_cans
World.update_yellow_can = update_yellow_can
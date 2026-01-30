import cv2
import numpy as np
from world import Can
from world.World import World

def rect_to_vertices(x: int, y: int, w: int, h: int) -> float:
    """
    Converts a rectangle of x, y, w, h format 
    to a rectangle in vertex format.
    
    :param x: X position of top-left corner of rectangle
    :param y: Y position of top-left corner of rectangle
    :param w: Width of rectangle
    :param h: Height of rectangle

    Returns:
    - Rectangle in ((x1, y1), (x2, y2), (x3, y3), (x4, y4)) format
    """
    return np.array([
        [x,     y],
        [x+w,   y],
        [x+w,   y+h],
        [x,     y+h]
    ])

def polygons_overlap(poly1: list | tuple, poly2: list | tuple) -> bool:
    """
    Checks if two polygons of vertex format overlap.
    
    :param poly1: first polygon in ((x1, y1), (x2, y2), (x3, y3), (x4, y4)) format
    :param poly2: second polygon in ((x1, y1), (x2, y2), (x3, y3), (x4, y4)) format

    Returns:
    - True if overlap, False otherwise
    """

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
    
    for axis in axes(poly1) + axes(poly2):
        p1 = project(poly1, axis)
        p2 = project(poly2, axis)
        if not overlap_1d(p1, p2):
            return False
    return True

def update_cans(self, mask, can_list, color) -> list:
    """
    Given a color range mask, find any cans, create a new can
    or update an existing can, and update the global can list.
    
    :param mask: Color range mask
    :param can_list: The appropriate can list to be updated
    :param color: The color of the can

    Returns:
    - Updated list of cans
    """
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
            tolerence = self.maslab.can_proximity_tolerence
            # Much higher tolerence if actively driving towards a can
            if self.maslab.robot.can_obligated:
                tolerence *= 10
            # Look through all existing cans to see if one should be updated or a new one created
            for i, old_can in enumerate(can_list):
                dist = np.sqrt((can.coords[0]-old_can.coords[0])**2 + (can.coords[1]-old_can.coords[1])**2)
                if dist<best_dist and dist<tolerence:
                    old_can.ticks_lost = 0
                    best_dist = dist
                    best_index = i
                # Remove a can if it hasn't been seen for ~1.5 minutes
                elif old_can.ticks_lost<1000:
                    old_can.ticks_lost += 1
                else:
                    old_can.confirmed = False
            if best_dist<tolerence:
                can_list[best_index].update(rect)
                replaced = True
            if not replaced:
                if self.maslab.robot.turn_factor<0.5:
                    can_list.append(can)
    return can_list
                
def update_red_cans(self, hsv):
    """
    Given an image, update the environment with new red cans.
    
    :param hsv: Image in HSV format.
    """
    # Hue / Saturation / Brightness ranges
    lower_red1 = np.array([0, 160, 40])
    upper_red1 = np.array([15, 255, 255])

    lower_red2 = np.array([165, 160, 40])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    self.red_cans = update_cans(self, mask, self.red_cans, Can.CanColor.RED)

def update_yellow_can(self, hsv):
    """
    Given an image, update the environment with new yellow cans.
    
    :param hsv: Image in HSV format.
    """
    # Hue / Saturation / Brightness ranges
    if not self.taken_yellow:
        lower_yellow = np.array([20, 200, 70])
        upper_yellow = np.array([35, 255, 255])

        mask_yellow = (cv2.inRange(hsv, lower_yellow, upper_yellow))

        self.yellow_cans = update_cans(self, mask_yellow, self.yellow_cans, Can.CanColor.YELLOW)

def update_green_cans(self, hsv):
    """
    Given an image, update the environment with new green cans.
    
    :param hsv: Image in HSV format.
    """
    # Hue / Saturation / Brightness ranges
    lower_green = np.array([40, 140, 50])
    upper_green = np.array([70, 255, 255])

    mask_green = (cv2.inRange(hsv, lower_green, upper_green))

    self.green_cans = update_cans(self, mask_green, self.green_cans, Can.CanColor.GREEN)


World.update_cans = update_cans                
World.update_red_cans = update_red_cans
World.update_green_cans = update_green_cans
World.update_yellow_can = update_yellow_can
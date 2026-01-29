import cv2
import math
import numpy as np
from world.World import World
from world.Goal import Goal

def get_length(line):
    return np.sqrt((line[3]-line[1])^2 + (line[2]-line[0])^2)

def get_bounds(self, hsv):
    # Blue color range
    lower_blue = np.array([90, 170, 100])
    upper_blue = np.array([120, 255, 255])

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Clean mask
    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

    # Create mask of blue pixels
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # Detect edges
    edges = cv2.Canny(blue_mask, 2, 3)
    # Detect lines
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=5,
        minLineLength=200,
        maxLineGap=100
    )

    # Find the longest line
    longest_line = (0, 0, 0, 0)
    if lines is not None:
        for line in lines:
            length = get_length(line[0])
            if length>get_length(longest_line):
                longest_line = line[0]
    
    self.bounds.update(*longest_line)

    intercept_form = None
    # Extrapolate line to edges of FOV
    if 0 not in longest_line:
        x1, y1, x2, y2 = longest_line
        if not (x2==x1):
            slope = (y2-y1)/(x2-x1)
            midpoint = (x1+(x2-x1)/2, y1+(y2-y1)/2)
            new_x1 = 0
            new_y1 = int(midpoint[1]-midpoint[0]*slope)
            new_x2 = self.maslab.video_width
            new_y2 = int(midpoint[1]+(self.maslab.video_width-midpoint[0])*slope)
            longest_line = (new_x1, new_y1, new_x2, new_y2)

            # Equation for line in y-intercept form
            intercept_form = (new_y1, slope)

    self.border = (intercept_form, longest_line)

    
def get_goal(self, hsv, frame, mask):
    # Clean up mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Get contours
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for cnt in contours:

        ########### Confirm shape ###########
        # Confirm area
        area = cv2.contourArea(cnt)
        if area < 2000:
            continue

        # Approximate polygon
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        # Must be quadrilateral
        if len(approx) != 4:
            continue

        return approx.reshape(4, 2)

    return None

def get_red_goal(self, hsv, frame):
    # Get mask
    lower_red1 = np.array([0, 150, 160])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 150, 160])
    upper_red2 = np.array([180, 255, 255])

    mask_red = (
        cv2.inRange(hsv, lower_red1, upper_red1) |
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    # Get goal
    goal = get_goal(self, hsv, frame, mask_red)
    if self.red_goal is not None:
        if goal is not None:
            self.red_goal.update(goal)
        else:
            self.red_goal.in_view = False
    elif goal is not None:
        self.red_goal = Goal(self, goal, min_area=700, max_area = 2000)
        check = self.red_goal.update(goal)
        if not check:
            self.red_goal = None
    
def get_green_goal(self, hsv, frame):
    # Get mask
    lower_green = np.array([40, 120, 50])
    upper_green = np.array([60, 255, 255])

    mask_green = (cv2.inRange(hsv, lower_green, upper_green))

    # Get goal
    goal = get_goal(self, hsv, frame, mask_green)
    if self.green_goal is not None:
        if goal is not None:
            self.green_goal.update(goal)
        else:
            self.green_goal.in_view = False
    elif goal is not None:
        self.green_goal = Goal(self, goal, min_area=700, max_area = 2000)
        check = self.green_goal.update(goal)
        if not check:
            self.green_goal = None
    
def get_yellow_goal(self, hsv, frame):
    # Get mask
    lower_yellow = np.array([20, 130, 150])
    upper_yellow = np.array([30, 255, 255])

    mask_yellow = (cv2.inRange(hsv, lower_yellow, upper_yellow))

    # Get goal
    goal = get_goal(self, hsv, frame, mask_yellow)
    if self.yellow_goal is not None:
        if goal is not None:
            self.yellow_goal.update(goal)
        else:
            self.yellow_goal.in_view = False
    elif goal is not None:
        self.yellow_goal = Goal(self, goal, min_area=20, max_area=500)
        check = self.yellow_goal.update(goal)
        if not check:
            self.yellow_goal = None
    
World.get_bounds = get_bounds
World.get_goal = get_goal
World.get_red_goal = get_red_goal
World.get_green_goal = get_green_goal
World.get_yellow_goal = get_yellow_goal
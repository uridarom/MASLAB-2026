import cv2
import math
import numpy as np
from world.World import World

def get_length(line):
    return np.sqrt((line[3]-line[1])^2 + (line[2]-line[0])^2)

def get_bounds(self, hsv):
    # Blue color range
    lower_blue = np.array([90, 100, 110])
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

World.get_bounds = get_bounds
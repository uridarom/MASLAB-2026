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

    lower_red1 = np.array([0, 150, 220])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 150, 220])
    upper_red2 = np.array([180, 255, 255])
    
def get_goal(self, hsv, frame):

    # Get mask
    lower_red1 = np.array([0, 150, 220])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 150, 220])
    upper_red2 = np.array([180, 255, 255])

    mask_red = (
        cv2.inRange(hsv, lower_red1, upper_red1) |
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    # Clean up
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

    # Get contours
    contours, _ = cv2.findContours(
        mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
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

        ########### Check if inside is dark ###########
        mask_shape = np.zeros(mask_red.shape, dtype=np.uint8)
        cv2.drawContours(mask_shape, [approx], -1, 255, -1)

        # Erode to avoid border
        inner_mask = cv2.erode(mask_shape, np.ones((10, 10), np.uint8))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        inside_pixels = gray[inner_mask > 0]

        # Confirm pixel values
        if np.mean(inside_pixels) > 150:
            continue

        ########### Check if outside is dark ###########
        dilated = cv2.dilate(mask_shape, np.ones((15, 15), np.uint8))
        outer_ring = dilated - mask_shape
        outside_pixels = gray[outer_ring > 0]

        # Confirm pixel values
        if np.mean(outside_pixels) > 150:
            continue

        return approx.reshape(4, 2)

    return None

World.get_bounds = get_bounds
World.get_goal = get_goal
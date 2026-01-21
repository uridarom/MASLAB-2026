import cv2
import numpy as np
from world import Can
from world.World import World

# Checks if box a and box b overlap
def boxes_overlap(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b

    return not (
        ax + aw < bx or
        bx + bw < ax or
        ay + ah < by or
        by + bh < ay
    )

# Creates video output and commands motors
def update_cans(self, hsv):

    # Hue / Saturation / Brightness ranges
    lower_red1 = np.array([0, self.maslab.color_tolerences[1], self.maslab.color_tolerences[2]])
    upper_red1 = np.array([self.maslab.color_tolerences[0], 255, 255])

    lower_red2 = np.array([180-self.maslab.color_tolerences[0], self.maslab.color_tolerences[1], self.maslab.color_tolerences[2]])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

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

        # Must fit reasonable aspect ratio range, unless cut off by FOV
        if y > 2 and x > 2 and x < self.maslab.video_width-2:
            aspect_ratio = h / float(w)
            if aspect_ratio > 1.50+self.maslab.ratio_tolerence or aspect_ratio < 1.50-self.maslab.ratio_tolerence:
                continue
        
        # How well contour fills the bounding box
        rect_area = w * h
        extent = area / rect_area
        if extent < 0.6:
            continue
        
        rectangles.append((x, y, w, h))
    
    for can in self.cans:
        can.in_view = False

    closest = (640, 0, 0, 0)
    # Check for overlap, remove smaller boxes if present
    for rect in rectangles:
        keep = True
        for ref_rect in rectangles:
            if boxes_overlap(rect, ref_rect):
                if rect[2]*rect[3] < ref_rect[2]*ref_rect[3]:
                    keep = False
                    break
        if keep:
            can = Can.Can(self, rect, Can.CanColor.RED)
            for i, old_can in enumerate(self.cans):
                if np.sqrt((can.coords[0]-old_can.coords[0])**2 + (can.coords[1]-old_can.coords[1])**2)<self.maslab.can_proximity_tolerence:
                    self.cans[i].update(rect)
            self.cans.append(can)
                
            # # Check if object is within bounds
            # lowest_point = get_lowest_point(rect)
            # point_color = (255, 0, 0)
            # in_bounds = True
            # if longest_line is not None:
            #     if lowest_point[1]<longest_line[0]+longest_line[1]*lowest_point[0]-self.maslab.boundry_padding:
            #         in_bounds = False
            #         point_color = (255, 0, 255)
            # closest_point = get_lowest_point(closest)
            # if lowest_point[1]>closest_point[1] and in_bounds:
            #     closest = rect
            
            # # Draw bounding box
            # cv2.rectangle(frame, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)

            # # Draw lowest point
            # cv2.circle(frame, lowest_point, 8, point_color, -1)
    
    
    # # Direct motors to go to point (if enabled)
    # if self.maslab.motor_action:
    #     turn_factor, distance, stopped, self.maslab = motor_control.go_to(closest_point[0], closest_point[1], self.maslab)
    #     # Show driving variables on video
    #     cv2.putText(frame, f"Turn Factor: {turn_factor:.4f}", (760, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    #     cv2.putText(frame, f"Distance: {distance}", (760, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


    # if self.maslab.motor_action:
    #     if stopped:
    #         start_pos = raven.get_motor_encoder(self.maslab.CHANNEL_2)
    #         while True:
    #             position = raven.get_motor_encoder(self.maslab.CHANNEL_2)
    #             diff = position - start_pos
    #             if diff == 440*self.maslab.roll_from_stop:
    #                 break
    #         print("Arrived At Object")
    #         break

World.update_cans = update_cans
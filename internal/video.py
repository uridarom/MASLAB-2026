import cv2
import numpy as np

def create_video_frame(frame, maslab, mouse):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Video frame
    cam_h = maslab.video_height
    cam_w = maslab.video_width
    # Map
    map_size = cam_h 
    boundry_size = cam_h-50
    robot_size = 35
    # Canvas
    canvas_h = cam_h+40
    canvas_w = cam_w+map_size+60
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
    canvas[:] = (255, 255, 255)

    padding = (canvas_h-cam_h)//2

    ############ Cans ############

    for can in maslab.world.cans:
        # Draw bounding box
        cv2.rectangle(frame, (can.rect[0], can.rect[1]), (can.rect[0] + can.rect[2], can.rect[1] + can.rect[3]), (0, 255, 0), 2)
        # Draw lowest point
        cv2.circle(frame, can.lowest_point, 8, (255, 0, 0) if can.in_bounds else (255, 0, 255), -1)
    
    ########### Border ###########

    # Draw border line
    cv2.line(frame, (maslab.world.border[1][0], maslab.world.border[1][1]), (maslab.world.border[1][2], maslab.world.border[1][3]), (255, 0, 255), 3)

    ########### Mouse ###########
    if mouse[0] >= 0 and mouse[1] >= 0 and mouse[0] < maslab.video_width and mouse[1] < maslab.video_height:
        h, s, v = hsv[mouse[1], mouse[0]]
        text = f"H:{h} S:{s} V:{v}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    ############ Map ############

    game_map = np.ones((map_size, map_size, 3), dtype=np.uint8) * 255
    # Draw competition borders
    cv2.rectangle(
        game_map,
        (map_size//2 - boundry_size//2, 0),
        (map_size//2 + boundry_size//2, boundry_size),
        (255, 75, 75),
        2
    ) 
    
    # Draw robot
    diff = (map_size-boundry_size)/2
    rot_rect = (((map_size-maslab.world.wheels.x)-(diff+robot_size/2), 
                    (map_size-maslab.world.wheels.y)-(diff+boundry_size/2)), 
                    (robot_size, robot_size), int(maslab.world.wheels.theta*180/np.pi))
    box = cv2.boxPoints(rot_rect)
    box = np.int32(box)
    cv2.drawContours(game_map, [box], 0, (0, 0, 0), -1)

    # Draw Text
    for i, val in enumerate((maslab.world.wheels.x, maslab.world.wheels.y, maslab.world.wheels.theta*180/np.pi)):
        cv2.putText(game_map, f"Y: {(val):.1f}", (padding+75*i, boundry_size+padding+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    
    ############ COMBINING ############

    # Update canvas
    canvas[padding:cam_h+padding, padding:cam_w+padding] = frame
    canvas[padding:map_size+padding, cam_w+padding:cam_w+map_size+padding] = game_map
    # Downsize to fit screen
    canvas = cv2.resize(canvas, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    
    return canvas
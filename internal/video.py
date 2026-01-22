import cv2
import numpy as np
from world import Can

def create_video_frame(frame, maslab, mouse):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Video frame
    cam_h = maslab.video_height
    cam_w = maslab.video_width
    # Map
    map_size = cam_h 
    boundry_size = cam_h-50
    # Canvas
    canvas_h = cam_h+40
    canvas_w = cam_w+map_size+60
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
    canvas[:] = (255, 255, 255)

    padding = (canvas_h-cam_h)//2

    ''' ============== Video ============== '''

    ############ Cans ############

    for can in maslab.world.cans:
        if can.in_view and can.confirmed:
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
    
    ''' ============== Map ============== '''

    # Create
    game_map = np.ones((map_size, map_size, 3), dtype=np.uint8) * 255
    diff = (map_size-boundry_size)/2
    def to_map_coords(coords):
        return (int(coords[0]+map_size/2+diff), int(-coords[1]+map_size-2*diff))

    ########### Borders ###########
    cv2.rectangle(
        game_map,
        (map_size//2 - boundry_size//2, 0),
        (map_size//2 + boundry_size//2, boundry_size),
        (255, 75, 75),
        2
    ) 

    ########### Cans ###########
    for can in maslab.world.cans:
        if can.confirmed:
            color_mod = 0 if can.in_view else -75
            # Draw can on map
            cv2.circle(game_map, (int(can.coords[0]+map_size/2+diff), map_size-int(can.coords[1])), 
                       16, (0, 255+color_mod, 0) if can.color==Can.CanColor.GREEN else (0, 0, 255+color_mod), -1)
    
    ########### Robot ###########

    # Create rotated triangle to represent robot
    w = maslab.robot_size
    h = maslab.robot_size * 1.2

    triangle = np.array([
        [-w/2,  h/2],
        [ w/2,  h/2],
        [ 0.0, -h/2],
    ], dtype=np.float32)

    theta = maslab.world.wheels.theta
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ], dtype=np.float32)

    robot_coords = to_map_coords((maslab.world.wheels.x, maslab.world.wheels.y))
    triangle = triangle @ R.T
    triangle[:, 0] += robot_coords[0]
    triangle[:, 1] += robot_coords[1]-maslab.robot_size*1.2/2

    cv2.drawContours(game_map, [triangle.astype(np.int32)], 0, (0, 0, 0), -1)

    ########### Text ###########
    cv2.putText(game_map, f"X: {(maslab.world.wheels.x):.1f}", (padding, boundry_size+padding+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    cv2.putText(game_map, f"Y: {(maslab.world.wheels.y):.1f}", (padding+120, boundry_size+padding+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    cv2.putText(game_map, f"O: {(maslab.world.wheels.theta*180/np.pi):.1f}", (padding+120*2, boundry_size+padding+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    cv2.putText(game_map, f"Turn Factor: {(maslab.robot.turn_factor):.4f}", (padding+120*3, boundry_size+padding+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

    ''' ============== Combining ============== '''

    # Update canvas
    canvas[padding:cam_h+padding, padding:cam_w+padding] = frame
    canvas[padding:map_size+padding, cam_w+padding:cam_w+map_size+padding] = game_map
    # Downsize to fit screen
    canvas = cv2.resize(canvas, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    
    return canvas
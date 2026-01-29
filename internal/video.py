import cv2
import numpy as np
from world import Can

# Expand a rotated rectangle by n_pixels amount
def expand_pts(pts, n_pixels):
    center = np.mean(pts, axis=0)
    expanded_pts = []
    for pt in pts:
        # Vector from center to point
        vec = pt - center
        # Normalize and scale
        dist = np.linalg.norm(vec)
        if dist == 0: continue
        new_pt = center + vec * (1 + n_pixels / dist)
        expanded_pts.append(new_pt)
    return np.array(expanded_pts, np.int32)

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

    for can_list in (maslab.world.red_cans, maslab.world.green_cans, maslab.world.yellow_cans):
        for can in can_list:
            if can.in_view and can.confirmed:
                # Draw bounding box
                cv2.rectangle(frame, (can.rect[0], can.rect[1]), (can.rect[0] + can.rect[2], can.rect[1] + can.rect[3]), can.get_color(), 2)
                # Draw lowest point
                cv2.circle(frame, can.lowest_point, 8, (255, 0, 0) if can.in_bounds else (255, 0, 255), -1)
    
    ########### Border ###########

    # Draw border line
    cv2.line(frame, (maslab.world.border[1][0], maslab.world.border[1][1]), (maslab.world.border[1][2], maslab.world.border[1][3]), (255, 0, 255), 3)

    ########### Mouse ###########
    mouse = (mouse[0]*2, mouse[1]*2)
    if mouse[0] >= 0 and mouse[1] >= 0 and mouse[0] < maslab.video_width and mouse[1] < maslab.video_height:
        h, s, v = hsv[mouse[1], mouse[0]]
        text = f"H:{h} S:{s} V:{v}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    ########### Status #############
    cv2.putText(frame, maslab.status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    ########### Goals ###########
    
     # Draw goals
    for goal in (maslab.world.red_goal, maslab.world.green_goal, maslab.world.yellow_goal):
        if goal is not None:
            if goal.in_view:
                cv2.drawContours(frame, [goal.quad], -1, (0, 255, 0), 3)

    ''' ============== Map ============== '''

    # Create
    game_map = np.ones((map_size, map_size, 3), dtype=np.uint8) * 255
    diff = (map_size-boundry_size)//2
    def to_map_coords_robot(coords):
        return (int(coords[0]*maslab.map_scale+map_size/2), int(-coords[1]*maslab.map_scale+map_size-2*diff))
    def to_map_coords_homography(coords):
        return int(coords[0]*maslab.map_scale+map_size/2), map_size-int(coords[1]*maslab.map_scale)

    ########### Grid ###########

    h = boundry_size
    w = boundry_size
    rows, cols = (int(22/maslab.map_scale), int(22/maslab.map_scale))
    dy, dx = h / rows, w / cols

    # Vertical lines
    for x in np.linspace(start=int(dx), stop=int(w-dx), num=cols-1):
        x = int(round(x))
        cv2.line(game_map, (x+diff, 0), (x+diff, boundry_size), color=(100, 100, 100), thickness=1)

    # Horizontal lines
    for y in np.linspace(start=dy, stop=h-dy, num=rows-1):
        y = int(round(y))
        cv2.line(game_map, (diff, y), (boundry_size+diff, y), color=(100, 100, 100), thickness=1)

    ########### Map Border ###########

    cv2.rectangle(
        game_map,
        (map_size//2 - boundry_size//2, 0),
        (map_size//2 + boundry_size//2, boundry_size),
        (0, 0, 0),
        2
    ) 

    ########### Bounds ###########

    for bound_line in maslab.world.bounds.lines:
        if bound_line.confirmed:
            # Extend line to fill FOV
            ext_1 = np.array(bound_line.p1) - bound_line.unit*1000
            ext_2 = np.array(bound_line.p2) + bound_line.unit*1000
            cv2.line(game_map, to_map_coords_homography(ext_1.astype(int).tolist()), to_map_coords_homography(ext_2.astype(int).tolist()), (255, 0, 0), 3)

    ########### Goals ###########

    # Draw goals
    goal_colors = ((0, 0, 255), (0, 255, 0), (0, 255, 255))
    for i, goal in enumerate((maslab.world.red_goal, maslab.world.green_goal, maslab.world.yellow_goal)):
        if goal is not None:
            pts = np.array([to_map_coords_homography(item) for item in goal.coords], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.fillConvexPoly(game_map, expand_pts(pts, 10), color=(0, 0, 0))
            cv2.polylines(game_map, [pts], isClosed=True, color=goal_colors[i], thickness=2)
            cv2.circle(game_map, np.array(goal.get_centroid()).astype(int).tolist(), 4, (255, 255, 255), 1)

    ########### Cans ###########

    for can_list in (maslab.world.red_cans, maslab.world.green_cans, maslab.world.yellow_cans):
        for can in can_list:
            if can.confirmed:
                # Draw can on map
                cv2.circle(game_map, to_map_coords_homography(can.coords), 16, can.get_color(), -1)
        
    ########### Robot ###########

    # Create rotated triangle to represent robot
    w = maslab.robot_size
    h = maslab.robot_size * 1.2

    triangle = np.array([
        [-w/2,  h/2],
        [ w/2,  h/2],
        [ 0.0, -h/2],
    ], dtype=np.float32)

    theta = maslab.theta
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ], dtype=np.float32)

    robot_coords = to_map_coords_robot((maslab.x, maslab.y))
    triangle = triangle @ R.T
    triangle[:, 0] += robot_coords[0]
    triangle[:, 1] += robot_coords[1]-maslab.robot_size*1.2/2

    cv2.drawContours(game_map, [triangle.astype(np.int32)], 0, (0, 0, 0), -1)

    ########### Text ###########
    cv2.putText(game_map, f"X: {(maslab.x):.1f}", (padding, boundry_size+padding+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    cv2.putText(game_map, f"Y: {(maslab.y):.1f}", (padding+120, boundry_size+padding+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    cv2.putText(game_map, f"O: {(maslab.theta*180/np.pi):.1f}", (padding+120*2, boundry_size+padding+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    cv2.putText(game_map, f"Turn Factor: {(maslab.robot.turn_factor):.4f}", (padding+120*3, boundry_size+padding+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

    ''' ============== Combining ============== '''

    # Update canvas
    canvas[padding:cam_h+padding, padding:cam_w+padding] = frame
    canvas[padding:map_size+padding, cam_w+padding:cam_w+map_size+padding] = game_map
    # Downsize to fit screen
    canvas = cv2.resize(canvas, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    
    return canvas
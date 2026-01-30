import numpy as np
import cv2
from world.World import World

# Calibration points in image coordinates
PTS_IMAGE_PLANE = [
    [617, 515],
    [619, 374],
    [621, 284],
    [305, 365],
    [209, 508],
    [1020, 526],
    [932, 383],
    [876, 290]
]

# Calibration points in real-world coordinates
PTS_GROUND_PLANE = np.array([
    [0, 3],
    [0, 4],
    [0, 5],
    [-1, 4],
    [-1, 3],
    [1, 3],
    [1, 4],
    [1, 5]
])*30.48

# Compute homography
np_pts_ground = np.array(PTS_GROUND_PLANE)
np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

np_pts_image = np.array(PTS_IMAGE_PLANE)
np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

h, err = cv2.findHomography(np_pts_image, np_pts_ground)

def transform_uv_to_xy(self, u: int, v: int, integer=False) -> float:
    """
    Transforms coordinates from camera frame to absolute
    real-world coordinates.
    
    :param u: In-image X coordinate
    :param v: In-image Y coordinate
    :param integer: Return as integer or not
    """
    x, y = transform_uv_to_xy_relative(self, u, v)

    x_world = -np.sqrt(x**2 + y**2)*np.sin(-self.maslab.theta - np.arctan(x/y)) + self.maslab.x
    y_world = np.sqrt(x**2 + y**2)*np.cos(-self.maslab.theta - np.arctan(x/y)) + self.maslab.y

    if integer:
        return [int(x_world), int(y_world)]

    return [x_world, y_world]

def transform_uv_to_xy_relative(self, u, v, integer=False) -> float:
    """
    Transforms coordinates from camera frame
    to real-world coordinates relative to the robot.
    
    :param u: In-image X coordinate
    :param v: In-image Y coordinate
    :param integer: Return as integer or not
    """
    homogeneous_point = np.array([[u], [v], [1]])
    xy = np.dot(h, homogeneous_point)
    scaling_factor = 1.0 / xy[2, 0]
    homogeneous_xy = xy * scaling_factor
    x = homogeneous_xy[0, 0]
    y = homogeneous_xy[1, 0]

    if integer:
        return [int(x), int(y)]

    return [x, y]

def transform_to_robot(self, x: float | int, y: float | int) -> float:
    """
    Transforms real-world X Y coordinates to relative
    to the robot's frame.
    
    :param x: Real world X coordinate
    :param y: Real world Y coordnate
    """
    s = np.sin(self.maslab.theta)
    c = np.cos(self.maslab.theta)

    # undo translation
    dx = x - self.maslab.x
    dy = y - self.maslab.y

    # undo rotation
    x_robot =  c * dx - s * dy
    y_robot =  s * dx + c * dy

    return x_robot, y_robot

World.transform_uv_to_xy = transform_uv_to_xy
World.transform_to_robot = transform_to_robot
World.transform_uv_to_xy_relative = transform_uv_to_xy_relative
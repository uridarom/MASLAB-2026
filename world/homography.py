import numpy as np
import cv2

# Calibration points in image coordinates
PTS_IMAGE_PLANE = [
    [615, 680],
    [616, 616],
    [616, 518],
    [478, 615],
    [751, 614],
    [388, 519],
    [842, 518],
    [271, 617],
]

# Calibration points in real world coordinates
PTS_GROUND_PLANE = [
    [0, 46.7],
    [0, 50.7],
    [0, 58.7],
    [-4, 50.7],
    [4, 50.7],
    [-8, 58.7],
    [8, 58.7],
    [-10, 50.7],
]

# Compute homography
np_pts_ground = np.array(PTS_GROUND_PLANE)
np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

np_pts_image = np.array(PTS_IMAGE_PLANE)
np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

h, err = cv2.findHomography(np_pts_image, np_pts_ground)

# Convert image coordinates to real world coordinates
def transform_uv_to_xy(u, v):
    homogeneous_point = np.array([[u], [v], [1]])
    xy = np.dot(h, homogeneous_point)
    scaling_factor = 1.0 / xy[2, 0]
    homogeneous_xy = xy * scaling_factor
    x = homogeneous_xy[0, 0]
    y = homogeneous_xy[1, 0]
    return x, y
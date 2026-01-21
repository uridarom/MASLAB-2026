import numpy as np
import cv2

# Calibration points in image coordinates
PTS_IMAGE_PLANE = [
    [654, 291],
    [654, 372],
    [654, 549],
    [133, 359],
    [279, 283],
    [1177, 385],
    [1028, 301],
]

# Calibration points in real world coordinates
PTS_GROUND_PLANE = [
    [1.0112375, 0.0000],
    [0.7064375, 0.0000],
    [0.4016375, 0.0000],
    [0.7064375, -0.3048],
    [1.0112375, -0.3048],
    [0.7064375, 0.3048],
    [1.0112375, 0.3048],
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
    x = homogeneous_xy[0, 0]*100
    y = homogeneous_xy[1, 0]*100
    return x, y


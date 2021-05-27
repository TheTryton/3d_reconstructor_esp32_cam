import numpy as np
import cv2 as cv
import math as m
import common_math
from main import DEV

RADIUS = 19.5
TRAIN_HEIGHT = 10.5
LAP_TIME = 15.134

CAMERA_MATRIX = np.array([[1.2213483657167785e+03, 0, 5.2038106031345080e+02],
                          [0, 1.2227670297198733e+03, 3.9953361672646463e+02],
                          [0, 0, 1]])

T = np.array([[1, 0, 0, RADIUS],
              [0, 1, 0, TRAIN_HEIGHT],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

Q = np.array([[1, 0, 0, 0],
              [0, -1, 0, 0],
              [0, 0, 30*0.05, 0],   # Focal length multiplication obtained experimentally.
              [0, 0, 0, 1]])


def reproject(disparity_frame, frame, travel_time, vertices_frame, mask, mode=DEV):
    # model_transform = common_math.create_model_transform_matrix(T, theta)
    Tr = np.array([0, TRAIN_HEIGHT, -10*RADIUS])
    Tr2 = np.array([0, TRAIN_HEIGHT, 0])
    theta = common_math.calculate_rotation_angle(travel_time, LAP_TIME)

    def Ry(phi):
        return np.array([[m.cos(phi), 0, m.sin(phi)],
                          [0, 1, 0],
                          [-m.sin(phi), 0, m.cos(phi)]])

    my_pos = np.array([Tr @ Ry(-theta), [0., 0., 0.], [0., TRAIN_HEIGHT, 0.]], dtype=float)
    my_color = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], dtype=float)
    rev_proj_matrix = Q

    points = cv.reprojectImageTo3D(disparity_frame, rev_proj_matrix)

    R = Ry(-theta)

    frame_shape = vertices_frame.shape
    vertices_frame = mask == 255

    output_points = points[vertices_frame]
    output_colors = frame[vertices_frame] / 255.

    C = np.array([-frame_shape[1] // 2, frame_shape[0] // 2, 0.])
    output_points = ((output_points + C) @ Ry(m.pi) - Tr2) @ R

    if mode == DEV:
        output_points = np.concatenate((output_points, my_pos), axis=0)
        output_colors = np.concatenate((output_colors, my_color), axis=0)

    return output_points, output_colors

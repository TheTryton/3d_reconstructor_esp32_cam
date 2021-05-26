import numpy as np
import cv2 as cv
import math as m
import common_math

RADIUS = 19.5
TRAIN_HEIGHT = 10.5
LAP_TIME = 9.134

CAMERA_MATRIX = np.array([[1.2213483657167785e+03, 0, 5.2038106031345080e+02],
                           [0, 1.2227670297198733e+03, 3.9953361672646463e+02],
                           [0, 0, 1]])

T = np.array([[1, 0, 0, RADIUS],
               [0, 1, 0, TRAIN_HEIGHT],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

Q = np.array([[1,0,0,0],
           [0,-1,0,0],
           [0,0,30*0.05,0], #Focal length multiplication obtained experimentally.
           [0,0,0,1]])
# [[1, 0, 0, -5.62774048e+02],
#                [0, -1, 0, -3.84172371e+02],
#                [0, 0, 5.5, 1.22276703e+03],
#                [0, 0, 9.22664093e-01, 0]])


def reproject(disparity_frame, frame, travel_time, vertices_frame):
    #model_transform = common_math.create_model_transform_matrix(T, theta)
    Tr = np.array([0, TRAIN_HEIGHT, -10*RADIUS])
    theta = common_math.calculate_rotation_angle(travel_time, LAP_TIME)

    def Ry(theta):
        return np.array([[m.cos(theta), 0, m.sin(theta)],
                          [0, 1, 0],
                          [-m.sin(theta), 0, m.cos(theta)]])

    def ReflectY():
        return np.array([[-1., 0., 0.],
                         [0., 1., 0.],
                         [0., 0., 1.]])

    my_pos = np.array([Tr @ Ry(-theta), Tr @ Ry(-theta+m.pi), [0., 0., 0.], [0., TRAIN_HEIGHT, 0.]], dtype=float)
    my_color = np.array([[1., 0., 0.], [1., 1., 1.], [0., 1., 0.], [0., 0., 1.]], dtype=float)
    disparity_shape = disparity_frame.shape
    roi_1, roi_2 = disparity_shape[0] // 16, disparity_shape[1] // 16

    if np.max(disparity_frame[(disparity_shape[0] // 2 - roi_1) : (disparity_shape[0] // 2 + roi_1),
              (disparity_shape[1] // 2 - roi_2) : (disparity_shape[1] // 2 + roi_2)]) == 0:
        return my_pos, my_color

    rev_proj_matrix = Q

    points = cv.reprojectImageTo3D(disparity_frame, rev_proj_matrix)
    mind = disparity_frame.min()
    maxd = disparity_frame.max()
    drange = maxd - mind
    realmin = mind + drange * 0.2
    realmax = maxd
    mask = (disparity_frame > realmin)  # & (normalized_disparity < realmax)

    R = Ry(-theta)

    vertices_frame = vertices_frame > 0

    output_points = points[vertices_frame]          # points[mask]
    output_colors = frame[vertices_frame] / 255     # frame[mask] / 255
    C = np.array([-frame.shape[0] // 2, frame.shape[1] // 2, 0.])
    output_points = ((output_points + C) @ Ry(m.pi) - Tr) @ R
    #

    return np.concatenate((output_points, my_pos), axis=0), \
           np.concatenate((output_colors, my_color), axis=0)

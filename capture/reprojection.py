import numpy as np
import cv2 as cv
import math as m
import common_math

RADIUS = 19.5
TRAIN_HEIGHT = 10.5
LAP_TIME = 9.134

CAMERA_MATRIX = np.matrix([[1.2213483657167785e+03, 0, 5.2038106031345080e+02],
                           [0, 1.2227670297198733e+03, 3.9953361672646463e+02],
                           [0, 0, 1]])

T = np.matrix([[1, 0, 0, RADIUS],
               [0, 1, 0, TRAIN_HEIGHT],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

Q = np.matrix([[1, 0, 0, -5.62774048e+02],
               [0, -1, 0, -3.84172371e+02],
               [0, 0, 0, 1.22276703e+03],
               [0, 0, 9.22664093e-01, 0]])


def reproject(disparity_frame, frame, travel_time):
    #model_transform = common_math.create_model_transform_matrix(T, theta)

    rev_proj_matrix = Q

    points = cv.reprojectImageTo3D(disparity_frame, rev_proj_matrix)
    mind = disparity_frame.min()
    maxd = disparity_frame.max()
    drange = maxd - mind
    realmin = mind + drange * 0.2
    realmax = maxd
    mask = (disparity_frame > realmin)  # & (normalized_disparity < realmax)

    Tr = np.array([0, TRAIN_HEIGHT, -RADIUS])
    theta = 0    # common_math.calculate_rotation_angle(travel_time, LAP_TIME)

    def Ry(theta):
        return np.matrix([[m.cos(theta), 0, m.sin(theta)],
                          [0, 1, 0],
                          [-m.sin(theta), 0, m.cos(theta)]])

    R = Ry(theta)

    output_points = points[mask]
    output_colors = frame[mask] / 255
    output_points = (output_points + Tr) @ R

    return output_points, output_colors

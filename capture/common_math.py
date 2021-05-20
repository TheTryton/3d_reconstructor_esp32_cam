import numpy as np
import math as m


def calculate_rotation_angle(current_time, total_time):
    return current_time / total_time * 2 * m.pi


def create_model_transform_matrix(T, theta):
    R = np.array([[m.cos(theta),    0,      m.sin(theta),       0],
                  [0,               1,      0,                  0],
                  [-m.sin(theta),   0,      m.cos(theta),       0],
                  [0,               0,      0,                  1]])
    return T @ R

import cv2 as cv
import numpy as np
import open3d as o3d
import math as m
import time


def Rx(theta):
    return np.matrix([[1, 0, 0],
                      [0, m.cos(theta), -m.sin(theta)],
                      [0, m.sin(theta), m.cos(theta)]])


def Ry(theta):
    return np.matrix([[m.cos(theta), 0, m.sin(theta)],
                      [0, 1, 0],
                      [-m.sin(theta), 0, m.cos(theta)]])


def Rz(theta):
    return np.matrix([[m.cos(theta), -m.sin(theta), 0],
                      [m.sin(theta), m.cos(theta), 0],
                      [0, 0, 1]])


def calc_theta(t, lap_time):
    return t / lap_time * 2 * m.pi


url = 'capture.mp4'
frame_length = 30 * 4
cframe = 0
delta_angle = 2 * m.pi / (frame_length - 1)
curr_angle = 0
vector = np.array([0, 0, 1])
imw = 640
imh = 480

all_points = np.ndarray(shape=(0, 3), dtype=float)
all_colors = np.ndarray(shape=(0, 3), dtype=float)
cap = cv.VideoCapture(url)
counter = 0
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
vis = o3d.visualization.Visualizer()
vis.create_window()
pcd = o3d.geometry.PointCloud()
added = False

dist = 20
lap = 3
t = 0

Q = np.matrix([[1.00000000e+00, 0.00000000e+00, 0.00000000e+00, -5.62774048e+02],
               [0.00000000e+00, 1.00000000e+00, 0.00000000e+00, -3.84172371e+02],
               [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.22276703e+03],
               [0.00000000e+00, 0.00000000e+00, 9.22664093e-01, -0.00000000e+00]])
start = time.time()

while (1):
    # Take each frame
    ret, frame = cap.read()
    end = time.time()
    t = (end - start)

    if ret:
        height, width = frame.shape[:2]
        y = (height - imh) // 2

        cropped = frame[y:y + imh, 0:width]

        left = cropped[:, 0:width // 2]  # cv.imread("left.png")#
        right = cropped[:, width // 2:width]  # cv.imread("right.png")#

        # height, width = left.shape[:2]

        imgL = cv.cvtColor(left, cv.COLOR_BGR2GRAY)
        left_RGB = cv.cvtColor(left, cv.COLOR_BGR2RGB)
        imgR = cv.cvtColor(right, cv.COLOR_BGR2GRAY)

        stereo = cv.StereoBM_create(
            numDisparities=64,
            blockSize=5)
        disparity = stereo.compute(imgL, imgR)
        normalized_disparity = cv.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv.NORM_MINMAX,
                                            dtype=cv.CV_8U)

        rev_proj_matrix = np.zeros((4, 4))

        left_mat = np.matrix([[1.2213483657167785e+03, 0, 5.2038106031345080e+02],
                              [0, 1.2227670297198733e+03, 3.9953361672646463e+02],
                              [0, 0, 1]])
        right_mat = np.matrix([[1.2213483657167785e+03, 0, 5.2038106031345080e+02],
                               [0, 1.2227670297198733e+03, 3.9953361672646463e+02],
                               [0, 0, 1]])

        cv.stereoRectify(cameraMatrix1=left_mat, cameraMatrix2=right_mat,
                         distCoeffs1=0, distCoeffs2=0,
                         imageSize=(width, height),
                         R=np.eye(3), T=np.array([1, 0., 0.]),
                         R1=None, R2=None,
                         P1=None, P2=None,
                         Q=rev_proj_matrix)

        # rev_proj_matrix = np.float32([  [1,0,0,0],
        #                                [0,-1,0,0],
        #                                [0,0,24*0.05,0], #Focal length multiplication obtained experimentally.
        #                                [0,0,0,1]])

        theta = calc_theta(t, lap)
        R = np.matrix([[m.cos(theta), 0, m.sin(theta)],
                        [0, 1, 0],
                        [-m.sin(theta), 0, m.cos(theta)]])

        T = np.array([0, 0 , dist])

        points = cv.reprojectImageTo3D(normalized_disparity, rev_proj_matrix)
        mind = normalized_disparity.min()
        maxd = normalized_disparity.max()
        drange = maxd - mind
        realmin = mind + drange * 0.2
        realmax = maxd
        mask = (normalized_disparity > realmin)  # & (normalized_disparity < realmax)

        output_points = points[mask]
        output_colors = left_RGB[mask] / 255
        output_points = (output_points + T) @ R
        if cframe < frame_length:
            all_points = np.concatenate((all_points, output_points[::10]), axis=0)
            all_colors = np.concatenate((all_colors, output_colors[::10]), axis=0)
        cframe = cframe + 1
        cv.imshow('left', left)
        cv.imshow('right', right)
        cv.imshow('disparity', normalized_disparity)

        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
        # if k == ord('s'):
        # output_file = 'reconstructed.ply'
        # create_output(output_points, output_colors, output_file)

        if counter == 0:
            pcd.points = o3d.utility.Vector3dVector(all_points)
            pcd.colors = o3d.utility.Vector3dVector(all_colors)
            if not added:
                added = True
                vis.add_geometry(pcd)
            else:
                vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()
    else:
        cap.set(cv.CAP_PROP_POS_FRAMES, 0)
    counter = (counter + 1) % 1
    curr_angle += delta_angle

cv.destroyAllWindows()

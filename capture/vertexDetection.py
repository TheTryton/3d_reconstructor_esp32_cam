import cv2 as cv
import random as rng
import numpy as np


def vertex_detection(frame):
    sigma = 3
    k = (sigma * 5) | 1

    frame_gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    frame_gray = cv.GaussianBlur(src=frame_gray, ksize=(k, k), sigmaX=sigma, sigmaY=sigma)
    cv.threshold(src=frame_gray, thresh=0, maxval=255, type=cv.THRESH_BINARY | cv.THRESH_OTSU, dst=frame_gray)
    edges = cv.Canny(frame_gray, threshold1=1, threshold2=2)
    contours, hierarchy = cv.findContours(edges, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_SIMPLE)

    frame_copy = np.copy(frame)

    for i in range(len(contours)):
        contours[i] = cv.approxPolyDP(contours[i], epsilon=3, closed=True)
        color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
        for point in contours[i]:
            cv.drawMarker(frame_copy, (point[0][0], point[0][1]), color=color, markerType=cv.MARKER_SQUARE, thickness=10)
        cv.drawContours(frame_copy, contours, i, color, 2, cv.LINE_8, hierarchy, 0)

    return frame_copy

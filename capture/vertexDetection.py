import cv2 as cv
import random as rng
import numpy as np


def vertex_detection(frame):
    sigma = 3
    k = (sigma * 5) | 1

    frame_gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    frame_gray = cv.GaussianBlur(src=frame_gray, ksize=(k, k), sigmaX=sigma, sigmaY=sigma)
    cv.threshold(src=frame_gray, thresh=0, maxval=255, type=cv.THRESH_BINARY | cv.THRESH_OTSU, dst=frame_gray)
    edges = cv.Canny(frame_gray, threshold1=50, threshold2=200)
    contours, hierarchy = cv.findContours(edges, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_SIMPLE)

    return frame_gray

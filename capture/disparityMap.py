import cv2 as cv
import numpy as np


def disparity_map(frame_left, frame_right, params):
    sigma = 3
    k = (sigma * 5) | 1

    g1 = np.copy(frame_left)
    g2 = np.copy(frame_right)

    g1 = cv.GaussianBlur(g1, (k, k), sigmaX=sigma, sigmaY=sigma)
    g2 = cv.GaussianBlur(g2, (k, k), sigmaX=sigma, sigmaY=sigma)

    g1 = cv.cvtColor(g1, cv.COLOR_BGR2GRAY)
    g2 = cv.cvtColor(g2, cv.COLOR_BGR2GRAY)

    sbm = cv.StereoBM_create(16 * params["numDisparities"], 2*params["blockSize"]+5)
    sbm.setPreFilterSize(params["filterSize"])
    sbm.setPreFilterCap(params["filterCap"])
    sbm.setMinDisparity(params["minDisparity"])
    sbm.setTextureThreshold(params["textureThreshold"])
    sbm.setUniquenessRatio(params["uniquenessRatio"])
    sbm.setSpeckleWindowSize(params["speckleWindowSize"])
    sbm.setSpeckleRange(params["speckleRange"])

    disparity = sbm.compute(g1, g2)
    disparity = cv.normalize(src=disparity, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U, dst=disparity)
    return disparity


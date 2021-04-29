import cv2
from urllib import request
import numpy as np

"""
url = 'http://172.18.209.139/cam-hi.jpg'

while True:
    imgResp=request.urlopen(url)
    imgNp=np.array(bytearray(imgResp.read()),dtype=np.uint8)
    img=cv2.imdecode(imgNp,-1)
    dst_gray, dst_color = cv2.pencilSketch(img, sigma_s=20, sigma_r=0.01, shade_factor=0.01)
    cv2.imshow('i', dst_gray)
    if ord('q')==cv2.waitKey(10):
        exit(0)
"""
url = 'http://172.18.209.139/cam.mjpeg'

stream = request.urlopen(url)
bytes = bytes()
while True:
    bytes += stream.read(1024)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes = bytes[b+2:]
        img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        dst_gray, dst_color  = cv2.pencilSketch(img, sigma_s=5, sigma_r=0.03, shade_factor=0.03)
        cv2.imshow('i', dst_gray)
        if cv2.waitKey(1) == 27:
            exit(0)

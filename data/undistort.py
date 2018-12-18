# -*- coding: utf-8 -*-
"""
Created on Fri Dec  7 14:52:01 2018

@author: maurice
"""
import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
import os
import glob
import sys


# You should replace these 3 lines with the output in calibration step
DIM=(720,480)
K=np.array([[225.01642957249587, 0.0, 363.1393234596647], [0.0, 204.81914558887172, 232.07055709869866], [0.0, 0.0, 1.0]])
D=np.array([[0.4365410760866138], [0.650649172834986], [-1.289300541531953], [0.6230311687587067]])
P=np.array([[204.06195671621887, 0.0, 363.3661972300628], [0.0, 185.74552845415002, 231.048300045872], [0.0, 0.0, 1.0]])

def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), P, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)
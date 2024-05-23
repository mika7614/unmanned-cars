#!/usr/bin/env python


import cv2
import numpy as np


def show_lane():
    cap = cv2.VideoCapture('/dev/video10')
    ret, img = cap.read()
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # kernel = np.ones((3,3), np.uint8)
    # gray_img = cv2.erode(gray_img, kernel, iterations=1)
    # origin_thr = np.zeros_like(gray_img)
    # origin_thr[(gray_img >= 125)] = 255
    # src_points = np.array([[3,570], [387,460], [906,452], [1041,485]], dtype="float32")
    # dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")
    # M = cv2.getPerspectiveTransform(src_points, dst_points)
    # binary_warped = cv2.warpPerspective(origin_thr, M, (1280, 720), cv2.INTER_LINEAR)
    cv2.imshow('binary_warped', gray_img)
    cv2.waitKey(1)

if __name__ == "__main__":
    while(1):
        show_lane()

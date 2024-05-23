#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import rospy
import cv2
import os
import numpy as np
from controller import simple_control
import collections

#from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class camera:
    def __init__(self):

        self.camMat = []
        self.camDistortion = []

        self.cap = cv2.VideoCapture('/dev/video10')
        #self.cap = cv2.VideoCapture('/home/pi/challenge_video.mp4')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
	
	    #self.imagePub = rospy.Publisher('images', Image, queue_size=1)
        self.cmdPub = rospy.Publisher('lane_vel', Twist, queue_size=1)
        self.cam_cmd = Twist()
        self.cvb = CvBridge()
        self.max_rad = np.deg2rad(45)
        #***************************************************#
        self.sp_ctrl = simple_control()
        self.sp_ctrl.hor_samples.append(640)
        #***************************************************#
    
    def __del__(self):
        print('******************* cap is released ********************')
        self.cap.release()

    def spin(self):
        base_y_upper = 450
        base_y_lower = 700
        base_x = 640
        filter_w = 20
        filter_h = 15
        square = 127.0
        half_w = filter_w // 2
        half_h = filter_h // 2
        thresh = 30
        ret, img = self.cap.read()
        if ret == True:
            print('Successfully received from lane camera!')
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            #hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            #gray_img = cv2.inRange(hsv_img, (25,30,200), (59,124,255))
            kernel = np.ones((3,3), np.uint8)
            gray_img = cv2.erode(gray_img, kernel, iterations=1)
            origin_thr = np.zeros_like(gray_img)
            origin_thr[(gray_img >= 180)] = 255
            origin_thr[(gray_img < 180)] = 0
            kernel = np.ones((filter_h, filter_w))
            kernel *=  (square / kernel.size)
            roi = origin_thr[base_y_upper:base_y_lower, :]
            feature_mapping = cv2.filter2D(roi/255, 1, kernel)
            # left_lane
            left_fm = feature_mapping[:, :base_x - 200]
            lane_l_inds = np.where(left_fm > thresh)
            if len(lane_l_inds[0]) > 20:
                cand_ly = int(np.mean(lane_l_inds[0])) + base_y_upper
                cand_lx = int(np.mean(lane_l_inds[1]))
                true_y = lane_l_inds[0] + base_y_upper
                cv2.rectangle(img, (cand_lx-50, cand_ly-36), (cand_lx+50, cand_ly+36), (0,255,0), thickness=5)
                # left_f_y
                a1, a0 = np.polyfit(true_y, lane_l_inds[1], 1)
                L_dir_x = int(a1*base_y_upper + a0)
                cv2.line(img, (L_dir_x, base_y_upper), (cand_lx, cand_ly), color=(200,255,0), thickness=5)
                
            # right_lane
            right_fm = feature_mapping[:, base_x + 200:]
            lane_r_inds = np.where(right_fm > thresh)
            if len(lane_r_inds[0]) > 20:
                cand_ry = int(np.mean(lane_r_inds[0])) + base_y_upper
                cand_rx = int(np.mean(lane_r_inds[1])) + base_x
                true_y = lane_r_inds[0] + base_y_upper
                true_x = lane_r_inds[1] + base_x
                cv2.rectangle(img, (cand_rx-50, cand_ry-36), (cand_rx+50, cand_ry+36), (0,0,255), thickness=5)
                # right_f_y
                a1, a0 = np.polyfit(true_y, true_x, 1)
                R_dir_x = int(a1*base_y_upper + a0)
                cv2.line(img, (R_dir_x, base_y_upper), (cand_rx, cand_ry), color=(200,0,255), thickness=5)
                
            if len(lane_l_inds[0]) > 20 and len(lane_r_inds[0]) > 20:
                if np.abs(cand_rx - cand_lx) < np.abs(R_dir_x - L_dir_x):
                    prev_dir_x = self.sp_ctrl.hor_samples[-1]
                    if np.abs(prev_dir_x - R_dir_x) < np.abs(prev_dir_x - L_dir_x):
                        dir_x = R_dir_x
                    else:
                        dir_x = L_dir_x
                elif cand_rx - cand_lx > 900:
                    dir_x = (L_dir_x + R_dir_x) // 2
                else:
                    dir_x = self.sp_ctrl.hor_samples[-1]
            elif len(lane_l_inds[0]) > 20 or len(lane_r_inds[0]) > 20:
                if len(lane_l_inds[0]) > 20:
                    dir_x = (L_dir_x + 1279) // 2
                else:
                    dir_x = R_dir_x // 2

            if len(lane_l_inds[0]) > 20 or len(lane_r_inds[0]) > 20:
                rad = np.arctan((base_x - dir_x) / (720.0 - base_y_upper))
                deg = np.rad2deg(rad)
                if rad > self.max_rad:
                    rad = self.max_rad
                elif rad < -self.max_rad:
                    rad = -self.max_rad
                #******************* simple control ******************#
                if len(self.sp_ctrl.hor_samples) < self.sp_ctrl.size:
                    self.sp_ctrl.hor_samples.append(dir_x)
                else :
                    dir_x = int(self.sp_ctrl.iterate(dir_x))
                #*****************************************************#
            else:
                dir_x = int(self.sp_ctrl.hor_samples[-1])
            #*****************************************************#
            rad = np.arctan((base_x - dir_x) / (720.0 - base_y_upper))
            compensate = (719 - base_y_upper) * np.sin(rad)
            safe_y = int(base_y_upper + np.abs(compensate))
            if np.rad2deg(abs(rad)) > 32:
                #compensate
                temp = 1 / (0.5 + safe_y / 720.0)
                offset = 12 * np.power(temp, 2)
                rad = np.sign(rad) * (np.abs(rad) + np.deg2rad(offset))       
            cv2.line(img, (base_x, 719), (dir_x, base_y_upper), color=(255,0,0), thickness=5)
            cv2.line(img, (0, safe_y), (1279, safe_y), color=(255,255,0), thickness=5)
            
            self.cam_cmd.angular.z = rad
            self.cmdPub.publish(self.cam_cmd)
	        #self.imagePub.publish(self.cvb.cv2_to_imgmsg(img))
            cv2.imshow('img', img)
            cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('lane_vel', anonymous=True)
    rate = rospy.Rate(10)
    
    try:
        cam = camera()
        print(rospy.is_shutdown())  # FALSE
        while not rospy.is_shutdown():
            cam.spin()            
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass

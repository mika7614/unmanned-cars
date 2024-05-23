#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

class rplidarDetector:
    def __init__(self):
        self.min_dist = 0
        self.has_obs = Bool()

        rospy.init_node('obstacle_detection', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('ooo', Bool, queue_size=1)
        self.pub_lf = rospy.Publisher('min_distance_left', Int32, queue_size=5)
        self.pub_rg = rospy.Publisher('min_distance_right', Int32, queue_size=5)
        print 'LiDAR is OK'


    def callback(self, msg):
        left_minIndex = 0
        right_minIndex = 0
        n = 3
        # goes throgh  array and find minimum  on the right 
        print len(msg.ranges)  #1440
        #for i in range (0,10):
         #   list(msg.ranges[i]) = 10000
        #for i in range(1430, 1440):
         #   list(msg.ranges[i]) = 10000
        for i in range (360,700):
            if msg.ranges[i] < msg.ranges[right_minIndex]:
                right_minIndex = i
        for i in range (740, 1080):
            if msg.ranges[i] < msg.ranges[left_minIndex] and msg.ranges[i] > 0.05:
               left_minIndex = i
        
        # calculate distance
        self.right_minIndex = msg.ranges[right_minIndex]
        self.left_minIndex = msg.ranges[left_minIndex]
        
	self.left_disList = n * [self.left_minIndex]
	self.right_disList = n * [self.right_minIndex]
		
	self.left_disList[0:n-1] = self.left_disList[1:n]
        self.left_disList[n-1] = self.left_minIndex
        self.left_disListsum = 0
		
	self.right_disList[0:n-1] = self.right_disList[1:n]
        self.right_disList[n-1] = self.right_minIndex
        self.right_disListsum = 0
        for i in self.left_disList:
            self.left_disListsum += i
	for i in self.right_disList:
            self.right_disListsum += i
    
        self.left_disListmean = self.left_disListsum / n
	self.right_disListmean = self.right_disListsum / n
       # print self.angleleft
	     
	global a,b
        #public message
        if self.right_disListmean < 1.2 :
            print "right_disListmean = "+ str(self.right_disListmean) + ", stop"
            #print "****** " + str(minIndex) + " *********"
            self.has_obs.data = True
            a = True
            #self.pub.publish(self.has_obs)
            self.pub_rg.publish(self.right_disListmean * 100)
        else:
            self.has_obs.data = False
            a = False
            #self.pub.publish(self.has_obs)
            self.pub_rg.publish(10000)


        if self.left_disListmean < 1.2 :
            print "left_disListmean = "+ str(self.left_disListmean) + ", stop"
            #print "****** " + str(minIndex) + " *********"
            self.has_obs.data = True
            b = True
            #self.pub.publish(self.has_obs)
            self.pub_lf.publish(self.left_disListmean * 100)
        else:
            self.has_obs.data = False
            b = False
            #self.pub.publish(self.has_obs)
            self.pub_lf.publish(10000)

        self.pub.publish(a and b)

if __name__ == '__main__':
    try:
        detector = rplidarDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


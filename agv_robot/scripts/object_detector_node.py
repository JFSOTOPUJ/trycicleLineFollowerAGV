#!/usr/bin/env python

import rospy
import sys
import cv2
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObjectDetector:
    def __init__(self):
        print('Object detector node initialized')

		# Subscribe to sensor topic
        self.image_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)

		# Create object detector topic entity
        self.detector_pub = rospy.Publisher("/object_detector/collision", Bool, queue_size=1)
        

    def scan_callback(self, msg):
        # Calculate max object direction in degree
        count = msg.scan_time / msg.time_increment
        
		# Get detection values between -90 to 90
        lidarRanges = msg.ranges[:89] + msg.ranges[int(count - 89):]
        obstacle = False
        for i in range(len(lidarRanges)):
            # If object are closer than 25 cm from sensor considerate obstacle
            if lidarRanges[i] < 0.30:
                obstacle = True
        self.detector_pub.publish(obstacle)

if __name__ =='__main__':
    rospy.init_node('object_detector')
    object_detector = ObjectDetector()
    rospy.spin()


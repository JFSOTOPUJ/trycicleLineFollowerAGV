#!/usr/bin/env python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera:
    def __init__(self):
        print('Camera node initialized')
        self.bridge = CvBridge()

        # Init video capture
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        # Init camera topic
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)

        while (1):
            try:
                # Read camera video capture
                _, frame = self.cap.read()

                # Publish video on camera topic
                image_message = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(image_message)

            except Exception as e:
                print('\nCamera node closed')
                break

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
    
if __name__ =='__main__':
    rospy.init_node('camera_node')
    camera = Camera()
    rospy.spin()



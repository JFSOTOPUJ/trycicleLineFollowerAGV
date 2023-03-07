#!/usr/bin/env python

import rospy
import sys
import cv2
import traceback
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool
from agv_robot.msg import TrycicleDrive

class LineDetector:
    def __init__(self):
        print("Initializing line detector node")
        self.bridge = CvBridge()

        # Subscribe to usb camera topic
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        
        # Publish yellow result topic
        #self.image_yellow_pub = rospy.Publisher("/line_detector/pross_guide_img", Image, queue_size=10)

        # Publish blue result topic
        #self.image_blue_pub = rospy.Publisher("/line_detector/pross_mark_img", Image, queue_size=10)

	# Publish green result topic
        #self.image_red_pub = rospy.Publisher("/line_detector/pross_red_mark_img", Image, queue_size=10)

	# Publish speed control topic
        self.drive_pub = rospy.Publisher('/line_detector/drive', TrycicleDrive, queue_size=1)

        # Publish blue mark topic
        self.blue_mark_pub = rospy.Publisher('/line_detector/blue_mark', Bool, queue_size=1)
	
	# Publish green mark topic
        self.red_mark_pub = rospy.Publisher('/line_detector/red_mark', Bool, queue_size=1)
        

    def image_callback(self, msg):
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            traceback.print_exc()
            rospy.logerr(e)
            rospy.logerr("CvBridge Error, skipped image frame!")

    def process_image(self, cv_image):
        
        # --Image Size 640x480--
        height = 480
        width = 640

	mask_y_margin = 50
	mask_x_margin = 100


        crop_height = int(height * 1/2)
        total_height = int(height - crop_height)
        crop_width = int(width*0)
        total_width = int(width - (crop_width*2))

        # Crop image
        crop_img = cv_image[crop_height:height, crop_width:(width-crop_width)]

        # Change color to hsv and blur
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
	hsv = cv2.blur(hsv, (9,9))

        # Define yellow mask parameters
        lower_yellow = np.array([10,100,0]) #90
        upper_yellow = np.array([180,255,255])

        # Define blue mask parameters
        lower_blue = np.array([100,100,0]) #70, 100, 0
        upper_blue = np.array([180,255,255])

	# Define red mask parameters
        lower_red = np.array([120,100,0]) #80, 150,0
        upper_red = np.array([180,255,255])

        # Mask the image for yellow color
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        red_mask = cv2.inRange(hsv, lower_red, upper_red)

        # Get center of the tape in yellow mask image
        yellow_m = cv2.moments(yellow_mask)
        try:
            yellow_cx, yellow_cy = yellow_m['m10']/yellow_m['m00'], yellow_m['m01']/yellow_m['m00']
        except ZeroDivisionError:
            yellow_cy, yellow_cx = total_height, total_width/2

        # Get center of the tape in blue mask image
        blue_m = cv2.moments(blue_mask)
        try:
            blue_cx, blue_cy = blue_m['m10']/blue_m['m00'], blue_m['m01']/blue_m['m00']
        except ZeroDivisionError:
            blue_cy, blue_cx = total_height, total_width/2

	# Get center of the tape in green mask image
        red_m = cv2.moments(red_mask)
        try:
            red_cx, red_cy = red_m['m10']/red_m['m00'], red_m['m01']/red_m['m00']
        except ZeroDivisionError:
            red_cy, red_cx = total_height, total_width/2

        # Cut on the mask image showing the tape of cv_image
        yellow_res = cv2.bitwise_and(crop_img, crop_img, mask= yellow_mask)
        blue_res = cv2.bitwise_and(crop_img, crop_img, mask= blue_mask)
	red_res = cv2.bitwise_and(crop_img, crop_img, mask= red_mask)

	#Paint a square in red mask image
	red_res = cv2.rectangle(red_res, ((total_width/2)-mask_x_margin,(total_height/2)-mask_y_margin), ((total_width/2)+mask_x_margin,(total_height/2)+mask_y_margin),(255,0,0),2)
        blue_res = cv2.rectangle(blue_res, ((total_width/2)-mask_x_margin,(total_height/2)-mask_y_margin), ((total_width/2)+mask_x_margin,(total_height/2)+mask_y_margin),(255,0,0),2)

        # Paint a circle in the center of the tape
        yellow_res = cv2.circle(yellow_res,(int(yellow_cx), int(yellow_cy)), 10,(0,0,255),-1)
	blue_res = cv2.circle(blue_res,(int(blue_cx), int(blue_cy)), 10,(0,0,255),-1)
	red_res = cv2.circle(red_res,(int(red_cx), int(red_cy)), 10,(0,0,255),-1)
        
        # Paint y axis in result image
        yellow_res = cv2.line(yellow_res,(int(total_width/2),0),(int(total_width/2),total_height),(0,0,255),1)

        # Paint x axis in result image
        yellow_res = cv2.line(yellow_res,(0,int(total_height/2)),(total_width,int(total_height/2)),(0,0,255),1)


        # Paint y line offset
        yellow_res = cv2.line(yellow_res,(int(yellow_cx),0),(int(yellow_cx),total_height),(255,0,255),1)

        # Paint x line offset
        yellow_res = cv2.line(yellow_res,(0,int(yellow_cy)),(total_width,int(yellow_cy)),(255,0,255),1)

        # Get m errors

        self.dir_error = yellow_cx - (total_width/2)    # Value between -320:320
        self.spd_error = yellow_cy - (total_height/2)   # Value between -120:120

        self.blue_x_error = blue_cx - (total_width/2)
        self.blue_y_error = blue_cy - (total_height/2)

	self.red_x_error = red_cx - (total_width/2)
        self.red_y_error = red_cy - (total_height/2)

        # Range of degrees in dynamixel direction motor 50, 250
        # m and b calculated with linear ecuation y1=50, y2=250, x1-320, x2=320
        dir_m = 0.3125
        dir_b = 150

        self.dir_target = (self.dir_error * dir_m) + dir_b 


        # If there is no yellow guide on image spd_target = 0
        if (self.spd_error == total_height/2 and self.dir_error == 0):
            self.spd_target = 0
        else:
            self.spd_target = 300

	# Send red mark value to topic
        if (self.red_x_error > -mask_x_margin and self.red_x_error < mask_x_margin and self.red_y_error > -mask_y_margin and self.red_y_error < mask_y_margin):
            	self.red_mark_pub.publish(True)
		self.blue_mark_pub.publish(False)
        else:
            	self.red_mark_pub.publish(False)
        	if (self.blue_x_error > -mask_x_margin and self.blue_x_error < mask_x_margin and self.blue_y_error > -mask_y_margin and self.blue_y_error < mask_y_margin):
            		self.blue_mark_pub.publish(True)
        	else:
            		self.blue_mark_pub.publish(False)

        # Publish results
	msg = TrycicleDrive()
	msg.speed = self.spd_target
	msg.direction = self.dir_target
	self.drive_pub.publish(msg)
        #image_yellow_message = self.bridge.cv2_to_imgmsg(yellow_res, "passthrough")
        #image_blue_message = self.bridge.cv2_to_imgmsg(blue_res, "passthrough")
	#red_img = self.bridge.cv2_to_imgmsg(red_res, "passthrough")
        #self.image_yellow_pub.publish(image_yellow_message)
        #self.image_blue_pub.publish(image_blue_message)
	#self.image_red_pub.publish(red_img)
        
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('line_detector')
    line_detector = LineDetector()
    rospy.spin()

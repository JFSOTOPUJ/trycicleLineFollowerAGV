#!/usr/bin/env python

import rospy
import sys
import cv2
import traceback
import numpy as np 
import time
from std_msgs.msg import Int32, Bool
from agv_robot.msg import TrycicleDrive

class Brain:
    def __init__(self):
        print("--Initializing Brain node--")

        # Declare control variables
        self.control =  False
        self.endpoint = False
	self.interception = False
        self.collision = False
        self.elevator_up = False
	self.elevator_running = False
	self.elevator_control = False

        self.control_state = 0
        self.control_command = 0

	self.current_route = 0
	self.delivering = True

        # Subscribe to User Control node topics
        rospy.Subscriber("/user_control/command", Int32, self.user_control_callback)
	rospy.Subscriber("/user_control/route", Int32, self.set_route_callback)

        # Subscribe to line detector node topics
        rospy.Subscriber("/line_detector/blue_mark", Bool, self.ld_blue_mark_callback)
	rospy.Subscriber("/line_detector/drive", TrycicleDrive, self.ld_drive_callback)
	rospy.Subscriber("/line_detector/red_mark", Bool, self.ld_red_mark_callback)

	# Subscribe to elevator node topics
        rospy.Subscriber("/elevator/elevator_up", Bool, self.el_elevator_up_callback)

        # Subscribe to object detector node topics
        rospy.Subscriber("/object_detector/collision", Bool, self.od_collision_callback)
        
        # Create Publish Trycicle controller topics entities
 	self.drive_pub = rospy.Publisher("/brain/drive", TrycicleDrive, queue_size=1)

        # Create Publish Elevator controller topics entities
        self.elevator_pub = rospy.Publisher("/brain/elevator", Bool, queue_size=1)
    

    def ld_drive_callback(self, msg):
        if not self.endpoint and not self.collision and not self.control and not self.interception and self.current_route > 0:
            self.drive_pub.publish(msg)
	elif self.control_command > 0 and self.control_state == 1:		#	--GO AHEAD--
	    self.drive_pub.publish(msg)
	elif self.control_command > 0 and self.control_state == 2:		#	--TURN LEFT--
	    new_msg = TrycicleDrive()
	    new_msg.speed = 0
	    new_msg.direction = 50
	    self.drive_pub.publish(new_msg)
	elif self.control_command > 0 and self.control_state == 3:		#	--TURN RIGHT--
	    new_msg = TrycicleDrive()
	    new_msg.speed = 0
	    new_msg.direction = 250
	    self.drive_pub.publish(new_msg)
	elif self.control_command > 0 and self.control_state == 4:		#	--TURN AROUND LEFT--
	    new_msg = TrycicleDrive()
	    new_msg.speed = 250
	    new_msg.direction = 50
	    self.drive_pub.publish(new_msg)
	elif self.control_command > 0 and self.control_state == 5:		#	--TURN AROUND RIGHT--
	    new_msg = TrycicleDrive()
	    new_msg.speed = 250
	    new_msg.direction = 250
	    self.drive_pub.publish(new_msg)
	elif self.control_command > 0 and self.control_state == 6 or self.current_route == 0:		#	--STOP--
	    new_msg = TrycicleDrive()
	    new_msg.speed = 0
	    new_msg.direction = 150
	    self.drive_pub.publish(new_msg)
        elif self.control_command > 0 and self.control_state == 7:
            new_msg = TrycicleDrive()
	    new_msg.speed = 1274
	    new_msg.direction = 150
	    self.drive_pub.publish(new_msg)
	elif self.control_command > 0 and self.control_state == 8:		#	--TURN AROUND LEFT 2--
	    new_msg = TrycicleDrive()
	    new_msg.speed = 250
	    new_msg.direction = 100
	    self.drive_pub.publish(new_msg)
	elif self.control_command > 0 and self.control_state == 9:		#	--TURN AROUND RIGHT 2--
	    new_msg = TrycicleDrive()
	    new_msg.speed = 250
	    new_msg.direction = 200
	    self.drive_pub.publish(new_msg)

    def set_route_callback(self, msg):
        self.control = True
	self.current_route = msg.data

    def el_elevator_up_callback(self, msg):
	self.elevator_pub.publish(self.elevator_control)
        if msg.data and  self.endpoint and self.control and not self.elevator_up:
            print('Elevator is Up')
            self.elevator_running = False
	    self.elevator_up = True
        elif not msg.data and  self.endpoint and self.control and self.elevator_up:
	    print('Elevator is Down')
	    self.elevator_running = False
	    self.elevator_up = False


    def ld_blue_mark_callback(self, msg):
        if msg.data and not self.endpoint and not self.control and not self.interception:
            print('Endpoint detected')
            self.endpoint = True
        elif msg.data and self.endpoint and not self.control and not self.interception:
            new_msg = TrycicleDrive()
            new_msg.speed = 0
            new_msg.direction = 150
            self.drive_pub.publish(new_msg)

    def ld_red_mark_callback(self, msg):
        if msg.data and not self.interception:
            	print('Interception detected')
            	self.interception = True
            	self.control_state = 6
            	self.control_command = 7
		if self.current_route == 1:
			if self.delivering:
        			print('Go ahead')
        			self.control_state = 1
			else:   
                    		print('Go ahaed')
		    		self.control_state = 1
		elif self.current_route == 2:
			if self.delivering:
				self.control_state = 6
				time.sleep(0.5)
				self.control_state = 2
				time.sleep(0.5)	
                    		print('Turn left')
            	    		self.control_state = 4
                    		time.sleep(0.75)
				self.control_state = 8
                    		time.sleep(0.9)
			else:
                    		self.control_state = 6
				time.sleep(0.5)
				self.control_state = 3
				time.sleep(0.5)	
                    		print('Turn left')
            	    		self.control_state = 5
                    		time.sleep(0.75)
				self.control_state = 9
                    		time.sleep(0.9)
                self.control_command = 0
        	time.sleep(1)
	elif not msg.data and self.interception:
		print('Interception passed')
		self.interception = False

    def od_collision_callback(self, msg):
        if msg.data and not self.control and not self.collision:
            print('Collision detected')
            self.collision = True
            new_msg = TrycicleDrive()
            new_msg.speed = 0
            new_msg.direction = 150
            self.drive_pub.publish(new_msg)
        elif not msg.data and self.collision:
            print('Collision solved')
            self.collision = False
    
    def user_control_callback(self, msg):
        print('User command recieved', msg.data)
        self.control_command = msg.data
        self.control_state = 0
        self.control = True
        if self. endpoint:
            if not self.collision:
                if not self.elevator_running:
                    if msg.data == 1:                                           # Up or Down elevator
                        print('Elevator working...')
                        self.elevator_running = True
                        self.elevator_control = not self.elevator_control
                    elif msg.data == 2:                                         # Turn left
                        print('Turning left')
                        self.control_state = 2
                        time.sleep(1)
                        print('Turning around')
                        self.control_state = 4
                        time.sleep(15)
                        print('Finishing secuence')
                        self.control_state = 6
                    elif msg.data == 3:                                         # Turn right
                        print('Turning right')
	                self.control_state = 3
                        time.sleep(1)
                        print('Turning around')
                        self.control_state = 5
                        time.sleep(15)
                        print('Finishing secuence')
                        self.control_state = 6
                    elif msg.data == 4:
                        print('Go fordward')
                        self.control_state = 7
                        time.sleep(8)
                        print('Finishing secuence')
                        self.control_state = 6
                    elif msg.data == 5: 
                        print('Taking distance from package')
                        self.control_state = 1
                        time.sleep(5)
                        print('Finishing secuence')
                        self.control_state = 6
                    elif msg.data == 6:
                        print('Initializing route')
                        self.endpoint = False
                        self.control = False
                        self.delivering = not self.delivering
                        if self.delivering:
                            self.current_route = 0
                            print('Select route')
                else:
                    print('Elevator is running')
                    self.control = False
            else:
                print('An obstacle prevents control of this vehicle')
                self.control = False
        else:
            print('The robot is not in endpoint')
            self.control= False





    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('brain')
    brain = Brain()
    rospy.spin()

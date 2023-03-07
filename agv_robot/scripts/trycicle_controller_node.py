#!/usr/bin/env python

import os
import rospy
import time
from dynamixel_sdk import *
from std_msgs.msg import Int32
from agv_robot.msg import TrycicleDrive
from Ax12 import Ax12

# Default setting
DXL1_ID                     = 1                 # Direction Dynamixel ID
DXL2_ID                     = 2                 # Speed Dynamixel ID
BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/dynamixel'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-

DXL_DEGREES_2_POS_UNIT = 0.29297

class TrycicleController:
    def __init__(self):
        #   --Open Dynamixel AX12 conection--
        Ax12.DEVICENAME = DEVICENAME
        Ax12.BAUDRATE = BAUDRATE
        Ax12.connect()

        #   --Create AX12 instances--
        self.dir_dxl =  Ax12(DXL1_ID)
        self.spd_dxl =  Ax12(DXL2_ID)

        #   --Enable Dynamixels torque--
        self.dir_dxl.enable_torque()
        self.spd_dxl.enable_torque()

        print("Initializing dynamixel controller node")

        #   --Subscribe to direction control topic--
        #rospy.Subscriber('/brain/trycicle_direction', Int32, self.set_direction_callback)
        
        #   --Subscribe to speed control topic--
        #rospy.Subscriber('/brain/trycicle_speed', Int32, self.set_speed_callback)

	#   --Subscribe to drive control topic--
        rospy.Subscriber('/brain/drive', TrycicleDrive, self.set_drive_callback)


    def set_direction_callback(self, data):
        dynamixel_pos = int(data.data / DXL_DEGREES_2_POS_UNIT)
        self.dir_dxl.set_goal_position(dynamixel_pos)
    
    def set_speed_callback(self, data):
        self.spd_dxl.set_moving_speed(data.data)

    def set_drive_callback(self, msg):
        self.spd_dxl.set_moving_speed(msg.speed)
	dynamixel_pos = int(msg.direction / DXL_DEGREES_2_POS_UNIT)
        self.dir_dxl.set_goal_position(dynamixel_pos)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('trycicle_controller')
    trycicle_controller = TrycicleController()
    rospy.spin()


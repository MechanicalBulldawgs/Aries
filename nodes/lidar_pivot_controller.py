#!/usr/bin/env python

import rospy
from  lib_robotis import USB2Dynamixel_Device, Robotis_Servo
from math import *

'''
This module is responsible for interfacing with lidar pivot motor.
'''

class lidar_pivot_controller(object):

	def __init__(self):
		'''
		lidar pivot controller constructor
		'''
		rospy.init_node("lidar_pivot_controller")
		self.dyn = USB2Dynamixel_Device(dev_name = "/dev/ttyUSB0", baudrate = 1000000)
		self.servo = Robotis_Servo(self.dyn, 1)

		cmd = None
		while (True):
			print("Current Angle: " + str(degrees(self.servo.read_angle())))
			cmd = float(raw_input("> "))
			if cmd == "q": break
			self.servo.move_angle(radians(cmd))

if __name__ == "__main__":
	controller = lidar_pivot_controller()
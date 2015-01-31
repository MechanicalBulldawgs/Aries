#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with lidar pivot motor.
'''

class lidar_pivot_controller(object):

	def __init__(self):
		'''
		lidar pivot controller constructor
		'''
		rospy.init_node("lidar_pivot_controller")
		pass

if __name__ == "__main__":
	controller = lidar_pivot_controller()
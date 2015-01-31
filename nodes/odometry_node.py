#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with the IMU and publishing odometry messages
'''

class odometry_node(object):

	def __init__(self):
		'''
		Odometry node constructor
		'''
		rospy.init_node("odometry_node")
		pass

if __name__ == "__main__":
	odom_node = odometry_node()
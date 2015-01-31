#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with lidar sensor and publishing laser scans.
'''

class lidar_node(object):

	def __init__(self):
		'''
		lidar node constructor
		'''
		rospy.init_node("lidar_node")
		pass

if __name__ == "__main__":
	node = lidar_node()
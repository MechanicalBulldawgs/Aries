#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with lidar pivot motor encoder and reporting pivot position.
'''

class lidar_pivot_encoder_node(object):

	def __init__(self):
		'''
		lidar pivot encoder node constructor
		'''
		rospy.init_node("lidar_pivot_encoder_node")
		pass

if __name__ == "__main__":
	node = lidar_pivot_encoder_node()
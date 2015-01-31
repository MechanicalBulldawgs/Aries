#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with conveyor encoder and publishing conveyor location/angle.
'''

class conveyor_encoder_node(object):

	def __init__(self):
		'''
		conveyor encoder node constructor
		'''
		rospy.init_node("conveyor_encoder_node")
		pass

if __name__ == "__main__":
	node = conveyor_encoder_node()
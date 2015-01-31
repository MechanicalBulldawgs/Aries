#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with hopper encoder and publishing hopper location/angle.
'''

class hopper_encoder_node(object):

	def __init__(self):
		'''
		hopper_encoder_node node constructor
		'''
		rospy.init_node("hopper_encoder_node")
		pass

if __name__ == "__main__":
	node = hopper_encoder_node()
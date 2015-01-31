#!/usr/bin/env python

import rospy

'''
This module is responsible for controlling hopper motor based on received hopper commands.
'''

class hopper_controller(object):

	def __init__(self):
		'''
		hopper_controller constructor
		'''
		rospy.init_node("hopper_controller")
		pass

if __name__ == "__main__":
	controller = hopper_controller()
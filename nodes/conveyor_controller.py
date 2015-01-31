#!/usr/bin/env python

import rospy

'''
This module is responsible for controlling conveyor motor given received conveyor commands.
'''

class conveyor_controller(object):

	def __init__(self):
		'''
		conveyor controller constructor
		'''
		rospy.init_node("conveyor_controller")
		pass

if __name__ == "__main__":
	controller = conveyor_controller()
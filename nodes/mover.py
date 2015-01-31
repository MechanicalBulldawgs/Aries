#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with drive motors
 and subscribing to cmd_vel topic for drive commands.
'''

class mover(object):

	def __init__(self):
		'''
		mover constructor
		'''
		rospy.init_node("mover")
		pass

if __name__ == "__main__":
	mover_node = mover()
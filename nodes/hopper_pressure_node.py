#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with hopper pressure sensor and publishing sensor readings.
'''

class hopper_pressure_node(object):

	def __init__(self):
		'''
		hopper pressure node constructor
		'''
		rospy.init_node("hopper_pressure_node")
		pass

if __name__ == "__main__":
	node = hopper_pressure_node()
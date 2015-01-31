#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with current sensor and publishing sensor readings.
'''

class current_sensor_node(object):

	def __init__(self):
		'''
		current sensor node constructor
		'''
		rospy.init_node("current_sensor_node")
		pass

if __name__ == "__main__":
	node = current_sensor_node()
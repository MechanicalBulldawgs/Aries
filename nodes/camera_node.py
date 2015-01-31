#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with camera and publishing camera frames.
'''

class camera_node(object):

	def __init__(self):
		'''
		Camera node constructor
		'''
		rospy.init_node("camera_node")
		pass

if __name__ == "__main__":
	cam_node = camera_node()
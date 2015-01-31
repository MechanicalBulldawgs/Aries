#!/usr/bin/env python

import rospy

'''
This module is responsible for interfacing with lcd screen display.
'''

class lcd_screen_manager(object):

	def __init__(self):
		'''
		lcd_screen_manager node constructor
		'''
		rospy.init_node("lcd_screen_manager")
		pass

if __name__ == "__main__":
	manager = lcd_screen_manager()
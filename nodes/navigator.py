#!/usr/bin/env python


'''
This module is responsible for sending Twist commands to robot.
Input: Goal, Obstacle Avoidance Scan
Output: Twist commands
'''

############################
# Global Variables
GOAL_TOPIC = "nav_goal"
############################

class ReactiveNavitor(object):

	def __init__(self):
		'''
		'''
		pass

if __name__ == "__main__":
	navigator = ReactiveNavitor()
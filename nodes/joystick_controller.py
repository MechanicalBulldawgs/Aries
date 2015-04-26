#!/usr/bin/env python
import rospy, atexit, signal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

'''
This module is used to convert joystick messages into appropriate affector commands.
'''

###################################
# Constants
# TODO: make this ROS parameters
MAX_LINEAR_SPEED = 0.75
MAX_ANGULAR_SPEED = 2
JOY_LINEAR_AXIS = 1
JOY_ANGULAR_AXIS = 0
###################################

class Joystick_Controller(object):
	'''
	'''
	def __init__(self):
		'''
		Joystick Controller constructor
		'''
		rospy.init_node("joystick_controller")
		self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

		rospy.Subscriber("joy", Joy, self.joy_callback)

		atexit.register(self._exit_handler)

	def joy_callback(self, data):
		'''
		Joy topic callback
		'''
		######
		# Build Twist message
		######
		twister = Twist()
		twister.angular.z = MAX_ANGULAR_SPEED * data.axes[JOY_ANGULAR_AXIS]
		twister.linear.x = MAX_LINEAR_SPEED * data.axes[JOY_LINEAR_AXIS]
		self.drive_pub.publish(twister)
		#####
		# TODO: make other control (hopper, conveyor, etc) messages
		#####


	def _exit_handler(self):
		'''
		This function is called on exit 
		'''
		exit()


if __name__ == "__main__":
	controller = Joystick_Controller()
	rospy.spin()
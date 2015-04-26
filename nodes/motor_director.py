#!/usr/bin/python

"""This node interfaces with an Arduino to control motors """

import rospy, serial, atexit
from geometry_msgs.msg import Twist

class motor_director(object):
	'''
	'''
	def __init__(self):
		
		rospy.init_node('motor_director')

		"""Attempt to get parameters from the ROS server and use them to initialize the list 
			of touch sensors and the connection to the Arduino"""

		port = rospy.get_param('ports/arduino', '/dev/ttyACM0')
		print("Connecting to Arduino on port: " + str(port))
		self.arduino = serial.Serial(port, 9600, timeout = 1)
		print("Connected to Arduino on port: " + str(port))
		rospy.Subscriber("cmd_vel", Twist, cmd_vel_cb)
		atexit.register(self._exit_handler)

		
	def cmd_vel_cb(self, data):
		self.arduino.write('X:'+str(data.linear.x)+'\n')
		self.arduino.write('Z:'+str(data.angular.z)+'\n')

	'''
	TODO: add subscribers and callbacks for the motors
	'''

	'''
	Formats for messages to the arduino

	[0-F]:[150:600]
	X:[linear velocity]
	Z:[angular velocity]
	'''
	
	def _exit_handle(self):
		self.arduino.close()

if __name__ == "__main__":
	try:
		ser = serial_server()
	except rospy.ROSInterruptException as er:
		rospy.logerr(str(er))
	else:
		ser.run()
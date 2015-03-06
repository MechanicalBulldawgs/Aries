#!/usr/bin/python

"""This node interfaces with an Arduino to read sensor data """

import rospy, tf, serial, atexit
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


class serial_server(object):
	"""This class publishes sensor data received from an attached Arduino.
		Sensor data is published in the run() function"""
	def __init__(self):
		
		rospy.init_node('serial_server')
		self.imu_pub = rospy.Publisher("imu", Imu)

		"""Attempt to get parameters from the ROS server and use them to initialize the list 
			of touch sensors and the connection to the Arduino"""

		port = rospy.get_param('ports/arduino', '/dev/ttyACM0')

		self.arduino = serial.Serial(port, 9600, timeout = 1)

		atexit.register(self._cleanup)


	def run(self):
		"""Main loop for publishing sensor data"""
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():

			if not self.arduino.inWaiting() > 0:
				continue

			self.handle_data(self.arduino.readline())
			rate.sleep()

	def handle_data(self, data):
		'''
		This function parses data received from the arduino board.
		data is the raw string data packet received from the board.
		'''
		good_imu_data = False
		pass
		
	
	def publish_imu(self, imu_data):
		"""given imu data from teensy, publish as ROS message
			Expects imu_data in the format: {"SENSOR_NAME": [SENSOR VALUE, SENSOR_VALUE, SENSOR_VALUE], "SENSOR_NAME": [SENSOR_VALUE,...], ...}

		"""
		pass


	def _cleanup(self):
		"""Called at exit to close connection to Arduino"""
		self.arduino.close()

if __name__ == "__main__":
	try:
		ser = serial_server()
	except rospy.ROSInterruptException as er:
		rospy.logerr(str(er))
	else:
		ser.run()
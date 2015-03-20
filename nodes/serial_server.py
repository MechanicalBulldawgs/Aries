#!/usr/bin/python

"""This node interfaces with an Arduino to read sensor data """

import rospy, tf, serial, atexit
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool


class serial_server(object):
	"""This class publishes sensor data received from an attached Arduino.
		Sensor data is published in the run() function"""
	def __init__(self):
		
		rospy.init_node('serial_server')
		self.imu_pub = rospy.Publisher("imu", Imu, queue_size = 10)
		self.hopper_status_pub = rospy.Publisher("hopper_status", Bool, queue_size = 10)

		"""Attempt to get parameters from the ROS server and use them to initialize the list 
			of touch sensors and the connection to the Arduino"""

		port = rospy.get_param('ports/arduino', '/dev/ttyACM0')
		print("Connecting to Arduino on port: " + str(port))
		self.arduino = serial.Serial(port, 9600, timeout = 1)
		print("Connected to Arduino on port: " + str(port))
		atexit.register(self._cleanup)


	def run(self):
		"""Main loop for publishing sensor data"""
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():

			if not self.arduino.inWaiting() > 0:
				rate.sleep()
				continue

			self.handle_data(self.arduino.readline())
			

	def handle_data(self, data):
		'''
		This function parses data received from the arduino board.
		data is the raw string data packet received from the board.
		data should be in the format: "@IMU &ORIENT$R:VALUE,P:VALUE,Y:VALUE&ACCEL$X:VALUE,Y:VALUE,Z:VALUE&GYRO$X:VALUE,Y:VALUE,Z:VALUE@SENSOR1 ..."
		'''
		good_imu_data = False
		good_dist_inter_data = False
		try:
			data = (data.replace("\n", "")).split("@")[1:]	# Split on '@' to separate sensor information from arduino
		except:
			rospy.logerr("Received some bad data from Arduino; Don't be alarmed. This happens. Trying again.")
			return
		#################################
		hopper_status = None
		#################################
		imu_sensors = {}
		# Format: {GYRO: [X, Y, Z], ACCEL: [X, Y, Z], ORIENT: [Roll, Pitch, Yaw]....}
		#################################

		# Loop through each sensor type's data from arduino
		for item in data:
			try:
				sensor_name = (item.split(" "))[0] # Grab the sensor name
				sensor_data = (item.split(" "))[1] # Grab the sensor data
			except:
				rospy.logerr("Caught packet at a bad time -- Can't parse.")
				continue
			if sensor_name == "IMU":
				##################################
				# Parse IMU data
				##################################
				try:
					sensor_data = (sensor_data.split("&"))[1:]
					# parse each individual sensor on IMU
					for imu_sensor in sensor_data:
						sensor_data = imu_sensor.split("$")
						sensor_name = sensor_data[0]
						sensor_data = sensor_data[1].split(",")
						imu_sensors[sensor_name] = []
						for val in sensor_data:
							val = float(val.split(":")[1])
							imu_sensors[sensor_name].append(val)
				except:
					rospy.logerr("Could not parse IMU sensor data from Arduino")
					continue
				else:
					good_imu_data = True
			elif sensor_name == "IR_INTER":
				##################################
				# Parse IR Distance interrupter data
				##################################
				hopper_status = sensor_data
				good_dist_inter_data = True

		#############################################
		# Publish data (but only if our data is all good)
		#############################################
		if good_imu_data:
			self.publish_imu(imu_sensors)
		if good_dist_inter_data:
			self.publish_hopper_status(hopper_status)
	
	def publish_hopper_status(self, hopper_status):
		'''
			Given hopper_status ("0" or "1"), publish ROS message over hopper_status topic.
		'''	
		good_stuff = True
		# Construct message
		status_msg = Bool()
		# Check given hopper status
		if hopper_status == "0":
			status_msg.data = False
		elif hopper_status == "1":
			status_msg.data = True 
		else:
			good_stuff = False
			rospy.logerr("Cannot publish badly formatted hopper status data (IR Distance Interrupter).  Will try again next go around.")
		# If all good, publish message
		if good_stuff:
			self.hopper_status_pub.publish(status_msg)
	
	def publish_imu(self, imu_data):
		"""given imu data from teensy, publish as ROS message
			Expects imu_data in the format: {"SENSOR_NAME": [SENSOR VALUE, SENSOR_VALUE, SENSOR_VALUE], "SENSOR_NAME": [SENSOR_VALUE,...], ...}

		"""
		good_imu = True
		# Set up messages
		imu_msg = Imu()

		now_time = rospy.Time.now()
		imu_msg.header.stamp = now_time
		imu_msg.header.frame_id = 'base_link'

		for key in imu_data.keys():
			if key == "ORIENT":
				try:
					roll = imu_data[key][0]
					pitch = imu_data[key][1]
					yaw = imu_data[key][2]
				except:
					good_imu = False
					rospy.logerr("Cannot publish badly formatted imu_data (ORIENTATION).  Will try again next go around.")
				q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
				imu_msg.orientation = Quaternion(*q)
			elif key == "GYRO":
				try:
					imu_msg.angular_velocity.x = imu_data[key][0] 
					imu_msg.angular_velocity.y = imu_data[key][1]
					imu_msg.angular_velocity.z = imu_data[key][2] 
				except:
					good_imu = False
					rospy.logerr("Cannot publish badly formatted imu_data (GYRO).  Will try again next go around.")
			elif key == "ACCEL":
				try:
					imu_msg.linear_acceleration.x = imu_data[key][0]  
					imu_msg.linear_acceleration.y = imu_data[key][1]
					imu_msg.linear_acceleration.z = imu_data[key][2] 
				except:
					good_imu = False
					rospy.logerr("Cannot publish badly formatted imu_data (ACCEL).  Will try again next go around.")
		if good_imu:
			self.imu_pub.publish(imu_msg)


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
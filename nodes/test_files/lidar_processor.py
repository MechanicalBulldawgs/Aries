#!/usr/bin/env python

import rospy, signal, atexit, math
from sensor_msgs.msg import LaserScan


'''
Proof of concept/sandbox module for trying out different ways to process lidar data.
'''

####### Default Global Values #######
SCAN_TOPIC = "scan"
#####################################

class LidarProcessor(object):

	def __init__(self):
		'''
		Lidar Processor constructor
		'''
		rospy.init_node("lidar_processor")
		rospy.Subscriber(SCAN_TOPIC, LaserScan, self.scan_callback)

		self.current_scan = LaserScan() # current scan message
		self.received_scan = False 		# True if we've received a new scan, false if not

		atexit.register(self._exit_handler)
		signal.signal(signal.SIGINT, self._signal_handler)

	def scan_callback(self, data):
		'''
		This function is called everytime a message is transmitted over /scan topic
		'''
		# Update current scan
		self.current_scan = data
		# Set received scan flag to True
		self.received_scan = True

	def run(self):
		'''
		Main work loop.
		'''
		rate = rospy.Rate(10)
		this_scan = []
		while not rospy.is_shutdown():
			if self.received_scan: 
				this_scan = self.current_scan
				self.process_scan(this_scan)
				exit()

			rate.sleep()

	def process_scan(self, scan_msg):
		'''
		given a scan msg, process 
		'''
		current_angle = scan_msg.angle_min   # current angle will always have current angle (in lidar space)
		print("=== Processing ===")
		for i in xrange(0, len(scan_msg.ranges)):
			print(str(i) + ": " + str(math.degrees(current_angle)))
			current_angle += scan_msg.angle_increment


	def _signal_handler(self, signal, frame):
		'''
		Called when ctr-c signal is received
		'''
		exit()

	def _exit_handler(self):
		'''
		Called on script exit 
		'''
		pass


if __name__ == "__main__":
	processor = LidarProcessor()
	processor.run()


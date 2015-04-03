#!/usr/bin/env python

import rospy, signal, atexit, math
from sensor_msgs.msg import LaserScan


'''
Proof of concept/sandbox module for trying out different ways to process lidar data.
'''

####### Default Global Values #######
SCAN_TOPIC = "scan"
POST_DISTS = 0.6096  # Distance posts are apart from one another on beacon
MAX_RANGE = 1 		 # Max Scan range to consider
#####################################

class LaserObject(object):
	def __init__(self):
		self.left_edge 	= None
		self.right_edge = None
		self.length 	= None
		self.centroid 	= None
	def process(self):
		self.length = self.right_edge - self.left_edge
		self.centroid = int(self.length / 2) + self.left_edge

class BeaconLocalizer(object):

	def __init__(self):
		'''
		Lidar Processor constructor
		'''
		rospy.init_node("beacon_localizer")
		rospy.Subscriber(SCAN_TOPIC, LaserScan, self.scan_callback)

		self.vis_scan_pub = rospy.Publisher("vis_scan", LaserScan, queue_size = 10)

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

			rate.sleep()

	def correct_dist(self, dist):
		'''
		Given a distance, clip distance to MAX_RANGE if longer than MAX_RANGE
		'''
		return dist if dist <= MAX_RANGE else MAX_RANGE

	def obj_dist(self, r_obj, l_obj, scan_msg):
		'''
		Given two objects (right object and left object), use law of cosines to calc 
		 distance between the two.
		'''
		r = scan_msg.ranges[r_obj.centroid]
		l = scan_msg.ranges[l_obj.centroid]
		theta = abs((l_obj.centroid * scan_msg.angle_increment) - (r_obj.centroid * scan_msg.angle_increment))
		dist = math.sqrt(math.pow(r, 2) + math.pow(l, 2) - (2 * r * l * math.cos(theta)))
		return dist

	def process_scan(self, scan_msg):
		'''
		given a scan msg, process 
		'''
		current_angle = scan_msg.angle_min   # current angle will always have current angle (in lidar space)

		##################################
		# Visualization message (used to visualize software imposed laser range limit)
		vis_scan = LaserScan()
		vis_scan = scan_msg
		vis_scan.range_max = MAX_RANGE
		self.vis_scan_pub.publish(vis_scan)
		##################################


		####################### 
		# Pick out objects from scan
		#######################
		l_edge = None
		r_edge = None
		last_point = 0
		edge_thresh = 0.05
		scan_obj = LaserObject()
		scan_objs = []
		print("======================")
		for i in xrange(0, len(scan_msg.ranges)):

			last_dist = self.correct_dist(scan_msg.ranges[last_point])
			cur_dist = self.correct_dist(scan_msg.ranges[i])

			change = cur_dist - last_dist
			if abs(change) > edge_thresh and change < 0:
				# found a left edge
				scan_obj = LaserObject()
				scan_obj.left_edge = i
			elif abs(change) > edge_thresh and change > 0:
				# found a right edge
				if scan_obj.left_edge != None:
					# make sure we've found a left edge before this right edge
					scan_obj.right_edge = i
					scan_obj.process()
					scan_objs.append(scan_obj)
					scan_obj = LaserObject()

			current_angle += scan_msg.angle_increment

			last_point = i

		######################
		# Find the two posts
		######################
		for r_obj in scan_objs:
			for l_obj in scan_objs:
				print("==== OBJ DIST ====")
				print("Right Obj: (Centroid: %d, Angle: %f)" % (r_obj.centroid, math.degrees(r_obj.centroid * scan_msg.angle_increment)))
				print("Left Obj: (Centroid: %d, Angle: %f)" % (l_obj.centroid, math.degrees(l_obj.centroid * scan_msg.angle_increment)))
				print("Distance: " + str(self.obj_dist(r_obj, l_obj, scan_msg)))
		


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
	processor = BeaconLocalizer()
	processor.run()


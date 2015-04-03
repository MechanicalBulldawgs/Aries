#!/usr/bin/env python

import rospy, signal, atexit, math
from sensor_msgs.msg import LaserScan


'''
Proof of concept/sandbox module for trying out different ways to process lidar data.
'''

####### Default Global Values #######
SCAN_TOPIC = "scan"
POST_DIST = 0.6096  	# Distance posts are apart from one another on beacon
POST_DIST_ERR = 0.025 	# Error allowed in post distance
MAX_RANGE = 1 		 	# Max Scan range to consider
LARGE_NUMBER = 9999999	# Arbitrarily large number
#####################################

class LaserObject(object):
	'''
	Data structure used to hold objects detected in laser scans 
	'''
	def __init__(self):
		self.left_edge 	= None  # index of left edge
		self.right_edge = None  # index of right edge
		self.length 	= None	# obj length
		self.centroid 	= None  # index of object centroid
		self.distance 	= None 	# centroid range in scan
		self.angle 		= None 	# centroid angle in scan

	def process(self, scan_msg):
		'''
		Given scan_msg and left and right edges, calculate:
		 - length
		 - centroid 
		 - distance 
		 - angle 
		'''
		assert self.left_edge != None and self.right_edge != None, "Object edges not set.  Cannot process."
		self.length = self.right_edge - self.left_edge
		self.centroid = int(self.length / 2) + self.left_edge
		self.distance = scan_msg.ranges[self.centroid]
		self.angle = self.centroid * scan_msg.angle_increment

class Beacon(object):
	'''
	Data structure used to store information on beacon found in laser scan 
	'''
	def __init__(self, right_post, left_post, actual_dist, err):
		self.right_post = right_post	# LaserObject that stores right post
		self.left_post 	= left_post		# LaserObject that stores left post
		self.actual_dist = actual_dist	# Actual distance between right and left post
		self.err = err 					# (POST_DIST) - |DIST(right_post, left_post)|


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

	def obj_dist(self, r_obj, l_obj):
		'''
		Given two objects (right object and left object), use law of cosines to calc 
		 distance between the two.
		'''
		r = r_obj.distance
		l = l_obj.distance
		theta = l_obj.angle - r_obj.angle
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
		l_edge = None		# Store left edge
		r_edge = None		# Store right edge
		last_point = 0
		edge_thresh = 0.05
		scan_obj = LaserObject()
		scan_objs = []
		print("======================")
		# loop through each laser scan
		for i in xrange(0, len(scan_msg.ranges)):
			# get corrected previous distance
			last_dist = self.correct_dist(scan_msg.ranges[last_point])
			# get corrected current distance
			cur_dist = self.correct_dist(scan_msg.ranges[i])
			# calculate change
			change = cur_dist - last_dist
			if abs(change) > edge_thresh and change < 0:
				# found a right edge
				scan_obj = LaserObject()
				scan_obj.right_edge = i
			elif abs(change) > edge_thresh and change > 0:
				
				# found a left edge
				if scan_obj.right_edge != None:
					# try to filter out noise
					if i - scan_obj.right_edge < 2:
						# reset scan_obj --> previous edge was just noise
						scan_obj = LaserObject()
					else:
						# make sure we've found a right edge already before this left edge
						scan_obj.left_edge = i
						scan_obj.process(scan_msg)
						scan_objs.append(scan_obj)
						scan_obj = LaserObject()
				else:
					scan_obj = LaserObject()

			# update current angle
			current_angle += scan_msg.angle_increment
			# update last point
			last_point = i

		######################
		# Find the two posts
		######################
		beacon = None
		min_beacon_err = LARGE_NUMBER
		for ri in xrange(0, len(scan_objs)):
			for li in xrange(ri  + 1, len(scan_objs)):
				r_obj = scan_objs[ri]
				l_obj = scan_objs[li]
				dist = self.obj_dist(r_obj, l_obj)
				if (dist > (POST_DIST - POST_DIST_ERR)) and (dist < (POST_DIST + POST_DIST_ERR)):
					beacon_err = abs(dist - POST_DIST)
					if beacon_err < min_beacon_err:
						beacon = Beacon(right_post = r_obj, left_post = l_obj, actual_dist = dist, err = beacon_err)
						min_beacon_err = beacon_err

				print("==== OBJ DIST ====")
				print("Right Obj: (Centroid: %d, Angle: %f)" % (r_obj.centroid, math.degrees(r_obj.angle)))
				print("Left Obj: (Centroid: %d, Angle: %f)" % (l_obj.centroid, math.degrees(l_obj.angle)))
				print("Distance: " + str(self.obj_dist(r_obj, l_obj)))
		if beacon != None:
			print("~~~ BEACON ~~~")
			print("(Centroid: %d, Angle: %f) ------ (Centroid %d, Angle %f)" % (beacon.left_post.centroid, beacon.left_post.angle, beacon.right_post.centroid, beacon.right_post.angle))
			print("Distance: " + str(beacon.actual_dist) + " (err: " + str(beacon.err) + ")")
		else:
			print("~~~ BEACON ~~~")
			print("Failed to find.")

		


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


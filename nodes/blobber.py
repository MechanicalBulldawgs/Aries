#!/usr/bin/env python

import rospy, cv2, atexit, signal
import numpy as np
import Image as Img
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


'''
This module is responsible for performing blob detection on images from camera sensor.
When significant blob is seen, publish a centroid.
'''

###### DEFAULT GLOBAL VALUES #######
SHOW_BLOB = False 	 
CAM_TOPIC = "cam_image"
CV_BRIDGE_IMAGE_ENCODING = "bgr8"
####################################

class Blobber(object):
	
	def __init__(self):
		'''
		Blobber node constructor
		'''
		rospy.init_node("blobber_node")

		self.bridge = CvBridge()
		self.lower_h = 110
		self.lower_s = 50
		self.lower_v = 50
		self.upper_h = 130
		self.upper_s = 255
		self.upper_v = 255

		atexit.register(self._exit_handler)
		signal.signal(signal.SIGINT, self._signal_handler)

		rospy.Subscriber(CAM_TOPIC, Image, self.img_callback)


	def img_callback(self, data):
		'''
		This function is called everytime a message is transmitted over the CAM_TOPIC topic.
		'''
		cv_img = self.bridge.imgmsg_to_cv2(data)
		hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
		# cv2.imshow("Blob", cv_img)
		# cv2.waitKey(3)

		# Define bounds of HSV color to look for
		# Note in general: Lower: [H - 10, 100, 100], Upper: [H + 10, 255, 255]
		lower_bound = np.array([self.lower_h, self.lower_s, self.lower_v])
		upper_bound = np.array([self.upper_h, self.upper_h, self.upper_v])

		# Threshold the HSV image to produce mask
		mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
		# cv2.imshow("Blob Mask", mask)
		# cv2.waitKey(3)

		# Bitwise-AND mask and original image
		result_img = cv2.bitwise_and(cv_img, cv_img, mask = mask)
		
		nonzero = np.nonzero(mask) 											# grab all indices where mask is nonzero
		hit_size = nonzero[0].size 											# the number of pixels that are within our target threshold
		hit_percent = float(hit_size)/(cv_img.shape[0] * cv_img.shape[1]) 	# percent of pixels within our target threshold

		# Calculate centroid of nonzero values in mask
		centroid = (int(sum(nonzero[1])/float(hit_size)), int(sum(nonzero[0])/float(hit_size))) if hit_size > 0 else (None, None)
		if not centroid[0] == None:
			# Draw a circle around centroid
			cv2.circle(result_img, centroid, radius = 10, color = (0, 255, 0), thickness = 1, lineType = 8, shift = 0)

		cv2.imshow("Blobber", result_img)
		cv2.waitKey(3)

	def run(self):
		'''
		'''
		rospy.spin()

	def _initialize_ros_params(self):
		'''
		'''
		global SHOW_BLOB, CAM_TOPIC, CV_BRIDGE_IMAGE_ENCODING
		pass

	def _signal_handler(self, signal, frame):
		'''
		This function is called when ctr-c signal is received.
		'''
		exit()

	def _exit_handler(self):
		'''
		'''
		try:
			cv2.destroyAllWindows()
		except:
			pass



if __name__ == "__main__":
	blobber = Blobber()
	blobber.run()
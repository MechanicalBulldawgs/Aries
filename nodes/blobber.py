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
		self.lower_s = 100
		self.lower_v = 100
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
		pass



if __name__ == "__main__":
	blobber = Blobber()
	blobber.run()
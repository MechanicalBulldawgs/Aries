#!/usr/bin/env python

import rospy, cv2, atexit, signal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

'''
This module is responsible for performing blob detection on images from camera sensor.
When significant blob is seen, publish a centroid.
'''

class Blobber(object):
	
	def __init__(self):
		'''
		Blobber node constructor
		'''
		
		rospy.init_node("blobber_node")

		self.bridge = CvBridge()

		atexit.register(self._exit_handler)
		signal.signal(signal.SIGINT, self._signal_handler)


	def img_callback(self, data):
		'''
		'''
		pass

	def run(self):
		'''
		'''
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
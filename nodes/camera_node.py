#!/usr/bin/env python

import rospy, cv2, atexit, signal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

'''
This module is responsible for interfacing with camera and publishing camera frames.
'''
###### DEFAULT GLOBAL VALUES #######
CAM_DEVICE_INDEX = 1 
SHOW_CAM = False 	 
CAM_TOPIC = "cam_image"
CV_BRIDGE_IMAGE_ENCODING = "bgr8"
####################################

class camera_node(object):

	def __init__(self):
		'''
		Camera node constructor
		'''

		rospy.init_node("camera_node")		# Register this as ROS node named 'camera_node'

		self.cap = None						# Will store opencv capture device
		self.bridge = CvBridge()			# Bridge used to convert CV images into ROS Image messages

		self._load_ros_parameters()
		self._initialize_camera()								# initialize camera
		atexit.register(self._exit_handler)						# Register function to be called upon exit
		signal.signal(signal.SIGINT, self._signal_handler)		# Register ctrl-c signal handler

		self.cap_pub = rospy.Publisher(CAM_TOPIC, Image, queue_size = 10)	# Create publisher for camera images

	def _load_ros_parameters(self):
		'''
		Calling this function tries to update globals for this module using ROS params 
		'''
		global CAM_TOPIC, CAM_DEVICE_INDEX, CV_BRIDGE_IMAGE_ENCODING, SHOW_CAM

		try:
			CAM_DEVICE_INDEX = 			rospy.get_param("/CAM_DEVICE_INDEX")
		except:
			pass
		try:
			SHOW_CAM = 					rospy.get_param("/DISPLAY_CAM_WINDOW")
		except:
			pass
		try:
			CAM_TOPIC = 				rospy.get_param("/CAM_TOPIC_NAME")
		except:
			pass
		try:
			CV_BRIDGE_IMAGE_ENCODING = 	rospy.get_param("/CV_BRIDGE_IMAGE_ENCODING")
		except:
			pass

	def _initialize_camera(self):
		'''
		This function is called to initialize the camera sensor.
		This function in a helper function for the camera node class.
		'''
		self.cap = cv2.VideoCapture(CAM_DEVICE_INDEX)
		
	def transmit_video(self):
		'''
		This function runs until ROS is shutdown or process is killed.  Continuously captures
		frames and publishes them over ROS topic.
		'''
		rate = rospy.Rate(50) # create a 10Hz rate

		while not rospy.is_shutdown():
			ret, frame = self.cap.read()	# Grab current frame from webcam

			# Only show cv2 image if show cam parameter is true
			if (SHOW_CAM):
				cv2.imshow("frame", frame)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break
			# publish frame as ROS Image message using cv_bridge
			try:
				self.cap_pub.publish(self.bridge.cv2_to_imgmsg(frame, CV_BRIDGE_IMAGE_ENCODING))
			except CvBridgeError, e:
				# TODO: log error
				print(e)

			rate.sleep()

	def _signal_handler(self, signal, frame):
		'''
		This function is called when ctr-c signal is received.
		'''
		print("received signal: " + str(signal))
		exit()

	def _exit_handler(self):
		'''
		This function is called on exit.
		'''
		# Release camera and destroy all cv2 windows on exit.
		try:
			self.cap.release()
			cv2.destroyAllWindows()
		except:
			pass


if __name__ == "__main__":
	cam_node = camera_node()
	cam_node.transmit_video()
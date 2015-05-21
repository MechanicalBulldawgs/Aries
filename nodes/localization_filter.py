#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import math, time

'''
Filter: 
'''

VALUESTOSTORE = 100
MAXDISTANCECHANGE = 0.5
FOUNDTIMEOUT = 15

class LocalizationFilter(object):

    def __init__(self):
        '''
        LocalizationFilter constructor
        '''
        rospy.init_node("localization_filter")
        ROBOPOSE_TOPIC = rospy.get_param("topics/localization_pose", "beacon_localization_pose")
        BEACON_LOST_TOPIC = rospy.get_param("topics/beacon_lost", "beacon_lost")
        ROBOPOSE_TOPIC_FILTERED = rospy.get_param("topics/localization_pose_filtered", "beacon_localization_pose_filtered")
		BEACON_LOST_TOPIC_FILTERED = rospy.get_param("topics/beacon_lost_filtered", "beacon_lost_filtered")

        rospy.subscriber(ROBOPOSE_TOPIC, PoseStamped, self.robotpose_callback)
        rospy.Subscriber(BEACON_LOST_TOPIC, Bool, self.beacon_lost_callback)


        self.pose_pub = rospy.Publisher(ROBOPOSE_TOPIC_FILTERED, PoseStamped, queue_size = 10)
		self.beacon_lost_pub = rospy.Publisher(BEACON_LOST_TOPIC_FILTERED, Bool, queue_size = 10)

        self.dataReady = False
        self.newPosition = None
        self.storedData = []
        self.beacon_lost = True
		self.PositionFilterQ_x = []
		self.PositionFilterQ_y = []
		self.BeconLostcouter = 0
		self.lastFound = 0

    def run(self):
        '''
        Main work loop.
        '''
        rate = rospy.Rate(10)
		
        while not rospy.is_shutdown():
            if self.dataReady:
				PositionFilterQ_x.append(self.newPosition.orientation.x)
				PositionFilterQ_y.append(self.newPosition.orientation.y)
				pose = PoseStamped()
				pose.pose.orientation.x = sum(self.PositionFilterQ_x) / (len(self.PositionFilterQ_x))
				pose.pose.orientation.y = sum(self.PositionFilterQ_y) / (len(self.PositionFilterQ_y))
				self.pose_pub.publish(pose)
				if (len(PositionFilterQ_x) > VALUESTOSTORE):
					self.PositionFilterQ_x.pop(0)
					self.PositionFilterQ_y.pop(0)
                self.dataReady = False
            rate.sleep()


    def robotpose_callback(self, data):
        self.newPosition = data.data
        self.dataReady = True
        
    def beacon_lost_callback(self, data):
        '''
        Callback for beacon_lost messages
        '''
		
        self.beacon_lost = data.data
		
		if((self.beacon_lost):
			pass
		else:
			self.lastFound = time.time()
			
		if (time.time() < self.lastFound + FOUNDTIMEOUT)):
			bacon_lost = Bool(True)
		else:
			bacon_lost = Bool(False)
			
		self.beacon_lost_pub.publish(bacon_lost)




if __name__ == "__main__":
    pass
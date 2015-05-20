#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import math

'''
Filter: 
'''

VALUESTOSTORE = 100
MAXDISTANCECHANGE = 0.5

class LocalizationFilter(object):

    def __init__(self):
        '''
        LocalizationFilter constructor
        '''
        rospy.init_node("localization_filter")
        ROBOPOSE_TOPIC = rospy.get_param("topics/localization_pose", "beacon_localization_pose")
        ROBOPOSE_TOPIC_FILTERED = rospy.get_param("topics/localization_pose_filtered", "beacon_localization_pose_filtered")

        rospy.subscriber(ROBOPOSE_TOPIC, PoseStamped, self.robotpose_callback)

        self.pose_pub = rospy.Publisher(ROBOPOSE_TOPIC_FILTERED, PoseStamped, queue_size = 10)

        self.dataReady = False
        self.newPosition = None
        self.storedData = []

    def run(self):
        '''
        Main work loop.
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.dataReady:
                self.dataReady = False
                tally = 0
                for each in range(0,VALUESTOSTORE):
                    currentCalculation = self.storedData[each]
                    xSquared = (currentCalculation.orientation.x - self.newPosition.orientation.x)**2
                    ySquared = (currentCalculation.orientation.y - self.newPosition.orientation.y)**2
                    distance = math.sqrt(xSquared + ySquared)
                    if distance < MAXDISTANCECHANGE:
                        tally += (each/100)

                if(tally >= VALUESTOSTORE/4):
                    self.storedData.append(self.newPosition)
                    self.storedData.pop(0)
                    self.pose_pub.publish(self.newPosition)


            rate.sleep()


    def robotpose_callback(self, data):
        self.newPosition = data.data
        self.dataReady = True


if __name__ == "__main__":
    pass
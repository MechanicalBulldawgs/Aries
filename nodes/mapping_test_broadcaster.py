#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose
import math

if __name__ == '__main__':
    rospy.init_node('map_to_base_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        br.sendTransform((2+2*math.sin(t), 2+2*math.cos(t), 0.14605), 
                     (0, 0, 0, 1),
                     rospy.Time.now(),
                     "base_link",
                     "map")
        rate.sleep()    

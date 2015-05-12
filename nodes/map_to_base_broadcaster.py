#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose

def handle_localization_pose(pose):
    br = tf.TransformBroadcaster()
    br.sendTransform((pose.position.x + WIDTH, pose.position.y, 0), 
                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "map")
    print "Publish"

if __name__ == '__main__':
    rospy.init_node('map_to_base_broadcaster')
    cellWidth = rospy.get_param("map_params/width")
    cellResolution = rospy.get_param("map_params/resolution")
    WIDTH = cellResolution*cellWidth
    
    ROBOPOSE_TOPIC = rospy.get_param("topics/localization_pose", "beacon_localization_pose")
    rospy.Subscriber(ROBOPOSE_TOPIC, Pose, handle_localization_pose)
    rospy.spin()
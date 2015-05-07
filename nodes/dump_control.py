"""Dumps the hopper while ensuring the collector is not in the way."""

import rospy
from std_msgs.msg import String 
from std_msgs.msg import Int16


class dump_control(object):
    """Subscribes to hopper/collector state topics to determine their position. Also publishes to a motor dump_control
    topics which affect the hopper/collector position"""

    rospy.init_node('dump_control')

    #Needs to publish to 3 topics. Dump Command Topic/Hopper State Topic/Collector State Topic
"""Dumps the hopper while ensuring the collector is not in the way."""

import rospy
from std_msgs.msg import String 
from std_msgs.msg import Int16


class hopp_coll_control(object):
    """Subscribes to hopper/collector state topics to determine their position. Also publishes to a motor dump_control
    topics which affect the hopper/collector position"""

    def __init__(self):

        #Initialize the node
        rospy.init_node('hopp_coll_control')


        #Also set up publisher to publish to command topics for collect and dump
        self.hopper_cmds_pub = rospy.Publisher("hopper_cmds", Int16, queue_size = 10)
        self.collector_spin_cmds_pub = rospy.Publisher("collector_spin_cmds", Int16, queue_size = 10)
        self.collector_tilt_cmds_pub = rospy.Publisher("collecter_tilt_cmds", Int16, queue_size = 10)
        

        #subcribe to state topics to figure out where we are in process
        rospy.Subscriber("hopper_state", String, self.hopp_state_callback)
        rospy.Subscriber("collector_state", String, self.coll_state_callback)
        rospy.Subscriber("scoop_safe_state", String, self.scoop_safe_state_callback)

        #subscribe to raw data topics to decide where we physically are
        rospy.Subscriber("hopper_pot", UInt16, self.hopper_callback)
        rospy.Subscriber("collector_pot", UInt16, self.collector_callback)
        rospy.Subscriber("scoop_safe", Bool, self.scoop_safe_callback)

            #Sets the data from the hopper_callback to data member
    def hopper_callback(self, angle):
        self.hopper_angle = angle.data

    #Sets the data from the collector_callback to data member
    def collector_callback(self, angle):
        self.collector_angle = angle.data

    #Sets the data from the IR distance interrupt to data member
    def scoop_safe_callback(self, dist):
    self.scoop_safe_dist = dist.data


    def hopp_state_callback(self, state):
        self.hopp_state = state.data




    #Needs to subscribe to 3 topics. Dump Command Topic/Hopper State Topic/Collector State Topic
"""Dumps the hopper while ensuring the collector is not in the way."""

import rospy
from std_msgs.msg import String 
from std_msgs.msg import Int16


class dump_control(object):
    """Subscribes to hopper/collector state topics to determine their position. Also publishes to a motor dump_control
    topics which affect the hopper/collector position"""

    def __init__(self):

                #Hopper Angle Constansts
        self.HOPPER_MAX = 180
        self.HOPPER_MIN = 75

        self.HOPPER_REST_MAX = 85
        self.HOPPER_DUMP_MIN = 175

        #Collector Angle Constants
        self.COLLECTOR_MAX = 300
        self.COLLECTOR_MIN = 245

        self.COLLECTOR_REST_MAX = 255
        self.COLLECTOR_DUMP_MIN = 295

        #Motor Command constants CHECK THESE VALUES!!!
        self.HOPPER_UP = 380
        self.HOPPER_DOWN = 360
        self.HOPPER_HALT = 370

        #VERFIY THESE VALUES
        self.COLLECTOR_BACK = 380
        self.COLLECTOR_FORWARD = 360
        self.COLLECTOR_HALT = 370

        #Initialize the node
        rospy.init_node('dump_control')

                # Initializes the hopper/collector angles to be considered in rest postion. This prevents log errors since the arduino 
        # does not immediatly send the angles. 
        self.hopper_angle = 80
        self.collector_angle = 250
        self.scoop_safe_bool = 1   #CHANGE THIS VALUE

        self.hopper_state = "Resting" #used for keeping track of previous state
        self.collector_state = "Resting"
        self.scoop_safe_state = "Safe"

        self.hopper_command = 370 #initial stall value, used for keeping track of hopper direction
        self.collector_spin_command = 370
        self.collector_tilt_command = 370


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

    def collector_state_callback(self, state):
        self.collector_state = state.data

    """def dump_hopper(self):
        #tilt collector back
        while self.collector_angle < COLLECTOR_DUMP_MIN:
            self.collector_spin_cmd = self.COLLECTOR_BACK

        #stop collector
        self.collector_spin_cmd = self.COLLECTOR_HALT

        #dump hopper and wait a few seconds
        while self.hopper_angle < self.HOPPER_DUMP_MIN:
            self.hopper_command = self.HOPPER_UP

        time.sleep(3)

        #put hopper back down
        while self.hopper_angle > self.HOPPER_MIN:
            self.hopper_command = self.HOPPER_DOWN

        #stop hopper
        self.hopper_command = self.HOPPER_HALT

        #put collector back
        while self.collector_angle > self.COLLECTOR_REST_MAX:
            self.collector_tilt_command = self.COLLECTOR_FORWARD

        #stop collector
        self.collector_tilt_command = self.COLLECTOR_HALT"""

    def publish_hopper_command(self, hopper_command):
        command_msg = Int16()
        command_msg.data = hopper_command
        self.hopper_cmds_pub.publish(command_msg) 

    def publish_collector_tilt_command(self, collector_tilt_command):
        collector_tilt_msg = Int16()
        collector_tilt_msg.data = collector_tilt_command
        self.collector_tilt_cmds_pub.publish(collector_tilt_msg)

    def run(self):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

                #tilt collector back
        while self.collector_angle < COLLECTOR_DUMP_MIN:
            self.publish_collector_tilt_command(self.COLLECTOR_BACK)

        #stop collector
        self.publish_collector_tilt_command(self.COLLECTOR_HALT)

        #dump hopper and wait a few seconds
        while self.hopper_angle < self.HOPPER_DUMP_MIN:
            self.publish_hopper_command(self.HOPPER_UP)

        time.sleep(3)

        #put hopper back down
        while self.hopper_angle > self.HOPPER_MIN:
            self.publish_hopper_command(self.HOPPER_DOWN)

        #stop hopper
        self.publish_hopper_command(self.HOPPER_HALT)

        #put collector back towards hopper
        while self.collector_angle > self.COLLECTOR_REST_MAX:
            self.publish_collector_tilt_command(self.COLLECTOR_FORWARD)

        #stop collector
        self.publish_collector_tilt_command(self.COLLECTOR_HALT)

        rate.sleep()

if __name__ == "__main__":
    try:
        Dump_Control = dump_control()
    except rospy.ROSInterruptException as er:
        rospy.logerr(str(er))
    else:
        Dump_Control.run()

    #Needs to subscribe to 3 topics. Dump Command Topic/Hopper State Topic/Collector State Topic
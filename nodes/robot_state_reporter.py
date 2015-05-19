#!/usr/bin/python

import rospy
from std_msgs.msg import UInt16, Int16, String
'''
Subscribes to IR Distance interrupter looking at scoops.  Publishes drive safety message concerning scoop.
Publishes state of hopper
Publishes state of collector
'''

class State_Reporter(object):

    def __init__(self):
        '''
        State reporter constructor
        '''
        rospy.init_node("robot_state_reporter")

        ############################
        # Load motor settings from parameter server
        ############################
        try:
            # HOPPER CONSTANTS
            self.hopper_max = rospy.get_param("dump_settings/hopper_min_angle")
            self.hopper_min = rospy.get_param("dump_settings/hopper_max_angle")
            
            self.dump_signal = int(rospy.get_param("dump_settings/dump_signal"))
            self.undump_signal = int(rospy.get_param("dump_settings/undump_signal"))
            self.hopper_stop_signal = int(rospy.get_param("dump_settings/stop_signal"))
            
            # Collector constants
            self.collector_max = rospy.get_param("collector_settings/collector_max_angle")
            self.collector_min = rospy.get_param("collector_settings/collector_min_angle")

            self.collector_spin_signal = int(rospy.get_param("collector_settings/spin_signal"))
            self.collector_unspin_signal = int(rospy.get_param("collector_settings/unspin_signal"))
            self.collector_spin_stop_signal = int(rospy.get_param("collector_settings/spin_stop_signal"))
            self.collector_tilt_signal = int(rospy.get_param("collector_settings/tilt_signal"))
            self.collector_untilt_signal = int(rospy.get_param("collector_settings/untilt_signal"))
            self.collector_tilt_stop_signal = int(rospy.get_param("collector_settings/tilt_stop_signal"))

        except:
            rospy.logerr("Failed to load motor settings from parameter server.")
            exit()

        self.hopper_state = None
        self.collector_state = None
        self.hopper_angle = None
        self.collector_angle = None

        # Load topics
        self.hopper_pot_topic = rospy.get_param("topics/hopper_potentiometer", "hopper_pot")
        self.collector_pot_topic = rospy.get_param("topics/collector_potentiometer", "collector_pot")
        hopper_state_topic = rospy.get_param("topics/hopper_state", "hopper_state")
        collector_state_topic = rospy.get_param("topics/collector_state", "collector_state")

        self.hopper_state_pub = rospy.Publisher(hopper_state_topic, String, queue_size = 10)
        self.collector_state_pub = rospy.Publisher(collector_state_topic, String, queue_size = 10)

        #Sets up subsribers to get the angles of the collector/hopper
        rospy.Subscriber(self.hopper_pot_topic, UInt16, self.hopper_callback)
        rospy.Subscriber(self.collector_pot_topic, UInt16, self.collector_callback)

    def run(self):
        '''
        '''
        rate = rospy.Rate(10)
        rospy.wait_for_message(self.hopper_pot_topic, UInt16)
        rospy.wait_for_message(self.collector_pot_topic, UInt16)
        while not rospy.is_shutdown():
            # Grab current angles
            cur_hopper_angle = self.hopper_angle
            cur_collector_angle = self.collector_angle 
            # Get current states
            self.hopper_state = self.get_hopper_state(cur_hopper_angle)
            self.collector_state = self.get_collector_state(cur_collector_angle)
            # Build state messages
            cstate_msg = String()
            cstate_msg.data = self.collector_state
            hstate_msg = String()
            hstate_msg.data = self.hopper_state 
            # publish states
            self.collector_state_pub.publish(cstate_msg)
            self.hopper_state_pub.publish(hstate_msg)
            rate.sleep()

    #Sets the data from the hopper_callback to data member
    def hopper_callback(self, angle):
        self.hopper_angle = angle.data

    #Sets the data from the collector_callback to data member
    def collector_callback(self, angle):
        self.collector_angle = angle.data

    def get_hopper_state(self, hopper_angle):
        '''
        Determine and return current hopper state 
        '''
        if hopper_angle <= self.hopper_min[1]: 
            state = "RESTING"
        elif self.hopper_min[1] < hopper_angle <= self.hopper_max[0]:
            state = "TRANSITIONING"
        elif self.hopper_max[0] < hopper_angle:
            state = "DUMPING"
            #rospy.logerr("Hopper Angle: Out of Bounds: " + str(hopper_angle) + ". Hopper may be in dangerous position.")
        return state

    def get_collector_state(self, collector_angle):
        '''
        Determine and return current collector tilt state 
        '''
        if collector_angle <= self.collector_min[0]: 
            state = "RESTING"
        elif self.collector_min[0] < collector_angle <= self.collector_max[0]:
            state = "TRANSITIONING"
        elif self.collector_max[0] < collector_angle:
            state =  "DUMPING"
            #rospy.logerr("Collector Angle: Out of Bounds: " + str(collector_angle) + ". Collector may be in dangerous position.")

        return state

if __name__ == "__main__":
    try:
        reporter = State_Reporter()
    except rospy.ROSInterruptException as er:
        rospy.logerr(str(er))
    else:
        reporter.run()

#!/usr/bin/python

import rospy

'''
This module listens for commands to dump, then handles coordinating the various motors for dumping.
Dumps the hopper while ensuring the collector is not in the way.
'''

class Dump_Controller(object):

    def __init__(self):
        '''
        Dump controller constructor.
        '''
        rospy.init_node("dump_controller")

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
        self.conllector_angle = None
        self.dump_duration = 5  # Duration (seconds) for waiting with dump at dump state for dirt to fall into collection bin

        # Load topics
        hopper_pot_topic = rospy.get_param("topics/hopper_potentiometer", "hopper_pot")
        collector_pot_topic = rospy.get_param("topics/collector_potentiometer", "collector_pot")
        hopper_state_topic = rospy.get_param("topics/hopper_state", "hopper_state")
        collector_state_topic = rospy.get_param("topics/collector_state", "collector_state")
        hopper_cmds_topic = rospy.get_param("topics/hopper_cmds", "hopper_control")
        collector_tilt_topic = rospy.get_param("topics/collector_tilt_cmds", "collector_tilt_control")
        # Setup subscriptions
        rospy.Subscriber(hopper_pot_topic, UInt16, self.hopper_pot_callback)
        rospy.Subscriber(collector_pot_topic, UInt16, self.collector_pot_callback)
        # Setup publishers
        self.hopper_state_pub = rospy.Publisher(hopper_state_topic, String, queue_size = 10)
        self.collector_state_pub = rospy.Publisher(collector_state_topic, String, queue_size = 10)
        self.hopper_cmds_pub = rospy.Publisher(hopper_cmds_topic, Int16, queue_size = 10)
        self.collector_tilt_pub = rospy.Publisher(collector_tilt_topic, Int16, queue_size = 10)

    def run(self):
        '''
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            ##############################
            # Orchestrate dump action
            ##############################
            #tilt collector back
            while self.collector_angle < self.collector_max[0]:
                self.publish_collector_tilt_command(self.collector_untilt_signal)
                rate.sleep()
            #stop collector
            self.publish_collector_tilt_command(self.collector_tilt_stop_signal)

            #dump hopper and wait a few seconds
            while self.hopper_angle < self.hopper_max[0]:
                self.publish_hopper_command(self.dump_signal)
                rate.sleep()

            rospy.sleep(self.dump_duration)

            #put hopper back down
            while self.hopper_angle > self.hopper_min[0]:
                self.publish_hopper_command(self.undump_signal)
                rate.sleep()
            #stop hopper
            self.publish_hopper_command(self.hopper_stop_signal)

            #put collector back towards hopper
            while self.collector_angle > self.collector_min[0]:
                self.publish_collector_tilt_command(self.collector_tilt_signal)
                rate.sleep()
            #stop collector
            self.publish_collector_tilt_command(self.collector_tilt_stop_signal)

            rate.sleep()

    def hopper_pot_callback(self, data):
        '''
        Callback function for hopper potentiometer data
        '''
        self.hopper_angle = data.data

    def collector_pot_callback(self, data):
        '''
        Callback function for collector tilt potentiometer data
        '''
        self.collector_angle = data.data

    def publish_hopper_command(self, hopper_command):
        command_msg = Int16()
        command_msg.data = hopper_command
        self.hopper_cmds_pub.publish(command_msg) 

    def publish_collector_tilt_command(self, collector_tilt_command):
        collector_tilt_msg = Int16()
        collector_tilt_msg.data = collector_tilt_command
        self.collector_tilt_cmds_pub.publish(collector_tilt_msg)

if __name__ == "__main__":
    try:
        controller = Dump_Controller()
    except rospy.ROSInterruptException as er:
        rospy.logerr(str(er))
    else:
        controller.run()


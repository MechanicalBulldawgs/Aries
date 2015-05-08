#!/usr/bin/env python

import rospy
from threading import Lock
from aries.msg import DurationCmd

'''
Module implements duration based teleoperation commands.

'''
###################################
# Global Constants
# Constants for hopper (150 - 600)
HOPPER_DUMP = 320  # value to send when dumping
HOPPER_STOP = 370  # value to send when stopping hopper
HOPPER_UNDUMP = 420 # value to send when returning to resting state
# Constants for conveyor spin (150 - 600)
COLLECTOR_SPIN = 320
COLLECTOR_STOP = 370
COLLECTOR_RSPIN = 420
# Constants for conveyor tilt (150 - 600)
COLLECTOR_TILT = 340
COLLECTOR_TSTOP = 370
COLLECTOR_UNTILT = 400
###################################

class Duration_Teleop(object):

    def __init__(self):
        '''
        duration teleop constructor
        '''
        rospy.init_node("duration_teleop")
        self.cmd_lock = Lock()      # Lock to protect command variables
        self.new_cmd = False        # Flag set when new message received
        self.current_cmd = None     # Stores current duration command

        # Load topic names
        self.cmds_topic           = rospy.get_param("topics/duration_cmds", "duration_cmds")
        drive_topic               = rospy.get_param("topics/drive_cmds", "cmd_vel")
        hopper_cmds_topic         = rospy.get_param("topics/hopper_cmds", "hopper_control")
        collector_spin_cmds_topic = rospy.get_param("topics/collector_spin_cmds", "collector_spin_control")
        collector_tilt_cmds_topic = rospy.get_param("topics/collector_tilt_cmds", "collector_tilt_control")     


        rospy.Subscriber(self.cmds_topic, DurationCmd, self.cmd_callback)

    def cmd_callback(self, data):
        '''
        Called everytime a DurationCmd message is received over cmds topic.
        '''
        with self.cmd_lock:
            self.new_cmd = True
            self.current_cmd = data

    def run(self):
        '''
        '''
        rospy.wait_for_message(self.cmds_topic, DurationCmd)
        rate = rospy.Rate(10)
        current_cmd = None
        start_time = 0 #rospy.get_time() -> get current time in float seconds
        while not rospy.is_shutdown():
            # Update variables safely
            with self.cmd_lock:
                new_cmd = self.new_cmd
                self.new_cmd = False
                current_cmd = self.current_cmd if new_cmd else current_cmd
            # if new cmd: interrupt last command (send stops)
            if new_cmd:
                # Send stops
                # Issue new command
                start_time = rospy.get_time()
            # otherwise, check duration, if under target duration, keep going else stop
            elif rospy.get_time() - start_time >= current_cmd.duration
                # Keep doing what we're doing
                # Calculate % left?
                pass


            rate.sleep()

    def robot_stop(self):
        '''
        Calling this function sends stop commands to all motors on robot
        '''
        pass





if __name__ == "__main__":
    pass
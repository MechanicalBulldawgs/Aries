#!/usr/bin/env python

import rospy
from threading import Lock
from aries.msg import DurationCmd
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String


'''
Module implements duration based teleoperation commands.

'''
###################################
# Global Constants
# Constants for hopper (150 - 600)
HOPPER_DUMP = -24   # value to send when dumping
HOPPER_STOP = 0   # value to send when stopping hopper
HOPPER_UNDUMP = 24 # value to send when returning to resting state
# Constants for conveyor spin (150 - 600)
COLLECTOR_SPIN = 24
COLLECTOR_STOP = 0
COLLECTOR_RSPIN = -24
# Constants for conveyor tilt (150 - 600)
COLLECTOR_TILT = -24
COLLECTOR_TSTOP = 0
COLLECTOR_UNTILT = 24
# Drive constants
DRIVE_SPEED = 40
DRIVE_STOP = 0
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

        # Load motor values
        global HOPPER_DUMP, HOPPER_STOP, HOPPER_UNDUMP 
        global COLLECTOR_SPIN, COLLECTOR_STOP, COLLECTOR_RSPIN 
        global COLLECTOR_TILT, COLLECTOR_TSTOP, COLLECTOR_UNTILT
        try:
            # Constants for hopper
            HOPPER_DUMP = int(rospy.get_param("dump_settings/dump_signal"))
            HOPPER_STOP = int(rospy.get_param("dump_settings/stop_signal"))
            HOPPER_UNDUMP = int(rospy.get_param("dump_settings/undump_signal"))
            # Constants for conveyor spin
            COLLECTOR_SPIN = int(rospy.get_param("collector_settings/spin_signal"))
            COLLECTOR_STOP = int(rospy.get_param("collector_settings/spin_stop_signal"))
            COLLECTOR_RSPIN = int(rospy.get_param("collector_settings/unspin_signal"))
            # Constants for conveyor tilt (150 - 600)
            COLLECTOR_TILT = int(rospy.get_param("collector_settings/tilt_signal"))
            COLLECTOR_TSTOP = int(rospy.get_param("collector_settings/tilt_stop_signal"))
            COLLECTOR_UNTILT = int(rospy.get_param("collector_settings/untilt_signal"))
        except:
            rospy.logerr("Failed to load motor parameters.")

        # Load topic names
        self.cmds_topic           = rospy.get_param("topics/duration_cmds", "duration_cmds")
        drive_topic               = rospy.get_param("topics/drive_cmds", "cmd_vel")
        hopper_cmds_topic         = rospy.get_param("topics/hopper_cmds", "hopper_control")
        collector_spin_cmds_topic = rospy.get_param("topics/collector_spin_cmds", "collector_spin_control")
        collector_tilt_cmds_topic = rospy.get_param("topics/collector_tilt_cmds", "collector_tilt_control") 
        dump_topic                = rospy.get_param("topics/dump_cmds", "dump_cmds")


        # Setup publishers
        self.drive_pub = rospy.Publisher(drive_topic, Twist, queue_size = 10)
        self.hopper_pub = rospy.Publisher(hopper_cmds_topic, Int16, queue_size = 10)
        self.collector_spin_pub = rospy.Publisher(collector_spin_cmds_topic, Int16, queue_size = 10)
        self.collector_tilt_pub = rospy.Publisher(collector_tilt_cmds_topic, Int16, queue_size = 10)
        self.dump_pub = rospy.Publisher(dump_topic, String, queue_size = 10)

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
        rate = rospy.Rate(5)
        current_cmd = None
        active = False
        start_time = 0 #rospy.get_time() -> get current time in float seconds
        while not rospy.is_shutdown():
            # Update variables safely
            with self.cmd_lock:
                new_cmd = self.new_cmd
                self.new_cmd = False
                current_cmd = self.current_cmd if new_cmd else current_cmd
                if current_cmd.cmd == "take-dump": current_cmd.duration = 0
            # if new cmd: interrupt last command (send stops)
            if new_cmd:
                print("New command received")
                active = True
                # Send stops
                self.robot_stop()
                # Get start time for new command
                start_time = rospy.get_time()
                # Issue new command
                self.issue_command(current_cmd.cmd)

            # otherwise, check duration, if over time stop
            elif (rospy.get_time() - start_time >= current_cmd.duration) and active:
                print("Current command expired")
                self.robot_stop()
                active = False
            elif active:
                # keep issuing current command
                self.issue_command(current_cmd.cmd)

            rate.sleep()

    def robot_stop(self):
        '''
        Calling this function sends stop commands to all motors on robot
        '''
        # Hopper stop
        hopper_stop = Int16()
        hopper_stop.data = HOPPER_STOP
        # Collector tilt stop
        collector_tilt_stop = Int16()
        collector_tilt_stop.data = COLLECTOR_TSTOP
        # Collector spin stop 
        collector_spin_stop = Int16()
        collector_spin_stop.data = COLLECTOR_STOP
        # Drive train stop
        drive_stop = Twist()
        drive_stop.linear.x = DRIVE_STOP
        drive_stop.angular.z = DRIVE_STOP
        # Publish messages
        self.drive_pub.publish(drive_stop)
        self.hopper_pub.publish(hopper_stop)
        self.collector_spin_pub.publish(collector_spin_stop)
        self.collector_tilt_pub.publish(collector_tilt_stop)
        
    def issue_command(self, cmd):
        '''
        Given string command, send appropriate motor commands
        Commands: forward, backward, left, right, collect, tilt-collector, dump, stop
        '''
        print("CMD: " + str(cmd))
        if cmd == "forward":
            twist = Twist()
            twist.linear.x = DRIVE_SPEED
            twist.angular.z = DRIVE_SPEED
            self.drive_pub.publish(twist)
        elif cmd == "backward":
            twist = Twist()
            twist.linear.x = -DRIVE_SPEED
            twist.angular.z = -DRIVE_SPEED
            self.drive_pub.publish(twist)
        elif cmd == "left":
            twist = Twist()
            twist.linear.x = DRIVE_SPEED
            twist.angular.z = -DRIVE_SPEED
            self.drive_pub.publish(twist)
        elif cmd == "right":
            twist = Twist()
            twist.linear.x = -DRIVE_SPEED
            twist.angular.z = DRIVE_SPEED
            self.drive_pub.publish(twist)
        elif cmd == "0":
            twist = Twist()
            twist.linear.x = DRIVE_SPEED
            self.drive_pub.publish(twist)
        elif cmd == "1":
            twist = Twist()
            twist.angular.z = DRIVE_SPEED
            self.drive_pub.publish(twist)
        elif cmd == "collect":
            collect_cmd = Int16()
            collect_cmd.data = COLLECTOR_SPIN
            self.collector_spin_pub.publish(collect_cmd)
        elif cmd == "uncollect":
            collect_cmd = Int16()
            collect_cmd.data = COLLECTOR_RSPIN
            self.collector_spin_pub.publish(collect_cmd)
        elif cmd == "tilt-collector":
            # this ignores duration? probably
            tilt_cmd = Int16()
            tilt_cmd.data = COLLECTOR_TILT
            self.collector_tilt_pub.publish(tilt_cmd)
        elif cmd == "untilt-collector":
            # this ignores duration? probably
            tilt_cmd = Int16()
            tilt_cmd.data = COLLECTOR_UNTILT
            self.collector_tilt_pub.publish(tilt_cmd)
        elif cmd == "dump":
            # this ignores duration? probably
            dump_cmd = Int16()
            dump_cmd.data = HOPPER_DUMP
            self.hopper_pub.publish(dump_cmd)
        elif cmd == "undump":
            dump_cmd = Int16()
            dump_cmd.data = HOPPER_UNDUMP
            self.hopper_pub.publish(dump_cmd)
        elif cmd == "stop":
            self.robot_stop()
        elif cmd == "take-dump":
            dump_cmd = String()
            dump_cmd.data = "DUMP"
            self.dump_pub.publish(dump_cmd)
        else:
            # invalid command
            pass




if __name__ == "__main__":
    teleop = Duration_Teleop()
    teleop.run()
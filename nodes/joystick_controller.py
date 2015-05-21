#!/usr/bin/env python
import rospy, atexit, signal, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, String

'''
This module is used to convert joystick messages into appropriate affector commands.
'''

###################################
# Constants
CONTROLLER_BUTTONS = {"A": 0, "B":1, "X": 2, "Y": 3, "R1": 5, "L1": 4, "BACK": 6, "START": 7} # TODO: FINISH THIS, USE BELOW
CONTROLLER_AXES = {"LSTICKV": 1, "LSTICKH": 0}
# TODO: make this ROS parameters
MAX_MAG = 7
SLOP_THRESH = 0.15
# Axes to use for drive twists
JOY_LINEAR_AXIS = CONTROLLER_AXES["LSTICKV"]
JOY_ANGULAR_AXIS = CONTROLLER_AXES["LSTICKH"]
JOY_DUMP_BTTN = CONTROLLER_BUTTONS["L1"]
JOY_UNDUMP_BTTN = CONTROLLER_BUTTONS["R1"]
JOY_CSPIN_BTTN = CONTROLLER_BUTTONS["A"]
JOY_CRSPIN_BTTN = CONTROLLER_BUTTONS["B"]
JOY_CTILT_BTTN = CONTROLLER_BUTTONS["X"]
JOY_CUTILT_BTTN = CONTROLLER_BUTTONS["Y"]
TAKE_DUMP_BTTN = CONTROLLER_BUTTONS["START"]
STOP_BTTN = CONTROLLER_BUTTONS["BACK"]

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
DRIVE_SPEED = 7
DRIVE_STOP = 0
TURN_SPEED = 7
MINING_DRIVE_SPEED = 7
ARC_DRIVE_SPEED = 7
###################################


class Joystick_Controller(object):
    '''
    Drive: Twist messages
    Conveyor spin: int between 150 - 600
    Conveyor tilt: int between 150 - 600
    Hopper dump: int between 150 - 600
    '''
    def __init__(self):
        '''
        Joystick Controller constructor
        '''
        rospy.init_node("joystick_controller")

        self.joy_received = False  

        global HOPPER_DUMP, HOPPER_STOP, HOPPER_UNDUMP 
        global COLLECTOR_SPIN, COLLECTOR_STOP, COLLECTOR_RSPIN 
        global COLLECTOR_TILT, COLLECTOR_TSTOP, COLLECTOR_UNTILT
        global DRIVE_SPEED, DRIVE_STOP, MINING_DRIVE_SPEED, ARC_TURN_SPEED, TURN_SPEED
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
            # Constants for drive speeds
            DRIVE_SPEED = int(rospy.get_param("drive_settings/drive_speed"))
            DRIVE_STOP = int(rospy.get_param("drive_settings/drive_stop"))
            MINING_DRIVE_SPEED = int(rospy.get_param("drive_settings/mining_drive_speed"))
            ARC_TURN_SPEED = int(rospy.get_param("drive_settings/arc_turn_speed"))
            TURN_SPEED = int(rospy.get_param("drive_settings/turn_speed"))
        except:
            rospy.logerr("Failed to load motor parameters.")  

        self.controller_state = Joy()

        self.prev_hopper_state = Int16()
        self.prev_collector_spin_state = Int16()
        self.prev_collector_tilt_state = Int16()
        self.prev_drive_state = Twist()

        # Load topic names
        self.joystick_topic       = rospy.get_param("topics/joystick", "joy")
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
        # Setup subscribers
        rospy.Subscriber(self.joystick_topic, Joy, self.joy_callback)

        atexit.register(self._exit_handler)

    def joy_callback(self, data):
        '''
        Joy topic callback
        '''
        self.controller_state = data
        self.joy_received = True


    def run(self):
        '''
        This function is the processing function for this module.
        '''
        rospy.wait_for_message(self.joystick_topic, Joy)  # Wait for messege on joy topic
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            # Grab most recent controller state
            current_state = self.controller_state
            ######
            # Build Twist message
            ######
            twister = Twist()
            #####
            # Get drive command
            #####
            lin_val = current_state.axes[JOY_LINEAR_AXIS]
            ang_val = current_state.axes[JOY_ANGULAR_AXIS]

            mag = math.sqrt(lin_val**2 + ang_val**2)
            if (lin_val <= SLOP_THRESH) and (lin_val >= -SLOP_THRESH):
                # Within 0 point slop
                lin_vel = 0
            else:   
                lin_vel = (lin_val / mag) * MAX_MAG

            if (ang_val <= SLOP_THRESH) and (ang_val >= -SLOP_THRESH):
                # Within 0 point slop
                ang_vel = 0
            else:   
                ang_vel = (ang_val / mag) * MAX_MAG

            twister.linear.x = lin_vel
            twister.angular.z = ang_vel
            #if (twister.linear.x != self.prev_drive_state.linear.x) or (twister.angular.z != self.prev_drive_state.angular.z):
            self.drive_pub.publish(twister)
            self.prev_drive_state = twister

            ###########################
            # Get Hopper command
            ##########################
            hopper_cmd = Int16()
            # get current button state (4 possible states)
            dump = True if current_state.buttons[JOY_DUMP_BTTN] == 1 else False
            undump = True if current_state.buttons[JOY_UNDUMP_BTTN] == 1 else False
            # determine what to do based on state
            if dump and not undump:
                hopper_cmd.data = HOPPER_DUMP
            elif not dump and undump:
                hopper_cmd.data = HOPPER_UNDUMP
            else:
                hopper_cmd.data = HOPPER_STOP 
            if hopper_cmd.data != self.prev_hopper_state.data:
                self.hopper_pub.publish(hopper_cmd)
            self.prev_hopper_state = hopper_cmd
            ###########################
            # Get Conveyor Spin command
            ##########################
            collector_cmd = Int16()
            spin = True if current_state.buttons[JOY_CSPIN_BTTN] == 1 else False
            rspin = True if current_state.buttons[JOY_CRSPIN_BTTN] == 1 else False
            if spin and not rspin:
                collector_cmd.data = COLLECTOR_SPIN
            elif not spin and rspin:
                collector_cmd.data = COLLECTOR_RSPIN
            else:
                collector_cmd.data = COLLECTOR_STOP
            if collector_cmd.data != self.prev_collector_spin_state.data:
                self.collector_spin_pub.publish(collector_cmd)
            self.prev_collector_spin_state = collector_cmd 
            ###########################
            # Get Conveyor tilt command
            ##########################
            tilt_cmd = Int16()
            tilt = True if current_state.buttons[JOY_CTILT_BTTN] == 1 else False 
            untilt = True if current_state.buttons[JOY_CUTILT_BTTN] == 1 else False
            if tilt and not untilt:
                tilt_cmd.data = COLLECTOR_TILT
            elif not tilt and untilt:
                tilt_cmd.data = COLLECTOR_UNTILT
            else:
                tilt_cmd.data = COLLECTOR_TSTOP
            
            if tilt_cmd.data != self.prev_collector_tilt_state.data:
                self.collector_tilt_pub.publish(tilt_cmd)
            self.prev_collector_tilt_state = tilt_cmd
            ############################
            # take dump command
            ############################
            if current_state.buttons[TAKE_DUMP_BTTN] == 1 and self.joy_received:
                take_dump_cmd = String()
                take_dump_cmd.data = "DUMP"
                self.dump_pub.publish(take_dump_cmd)

            if current_state.buttons[STOP_BTTN] == 1 and self.joy_received:
                take_dump_cmd = String()
                take_dump_cmd = "STOP"
                self.dump_pub.publish(take_dump_cmd)
                self.robot_stop()
            self.joy_received = False
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

    def _exit_handler(self):
        '''
        This function is called on exit 
        '''
        exit()

if __name__ == "__main__":
    controller = Joystick_Controller()
    controller.run()

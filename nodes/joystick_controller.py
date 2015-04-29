#!/usr/bin/env python
import rospy, atexit, signal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

'''
This module is used to convert joystick messages into appropriate affector commands.
'''

###################################
# Constants
CONTROLLER_BUTTONS = {"A": 0, "B":1, "X": 2, "Y": 3, "R1": 5, "L1": 4} # TODO: FINISH THIS, USE BELOW
# TODO: make this ROS parameters
MAX_LINEAR_SPEED = 0.75
MAX_ANGULAR_SPEED = 2
MAX_TANK_SPEED = 255
# Axes to use for drive twists
JOY_LINEAR_AXIS = 1
JOY_ANGULAR_AXIS = 0
JOY_TANK_L_AXIS = 1
JOY_TANK_R_AXIS = 4
JOY_DUMP_BTTN = CONTROLLER_BUTTONS["R1"]
JOY_UNDUMP_BTTN = CONTROLLER_BUTTONS["L1"]
JOY_CSPIN_BTTN = CONTROLLER_BUTTONS["A"]
JOY_CRSPIN_BTTN = CONTROLLER_BUTTONS["B"]
JOY_CTILT_BTTN = CONTROLLER_BUTTONS["X"]
JOY_CUTILT_BTTN = CONTROLLER_BUTTONS["Y"]
# Constants for hopper (150 - 600)
HOPPER_DUMP = 340  # value to send when dumping
HOPPER_STOP = 370  # value to send when stopping hopper
HOPPER_UNDUMP = 400 # value to send when returning to resting state
# Constants for conveyor spin (150 - 600)
COLLECTOR_SPIN = 340
COLLECTOR_STOP = 370
COLLECTOR_RSPIN = 400
# Constants for conveyor tilt (150 - 600)
COLLECTOR_TILT = 340
COLLECTOR_TSTOP = 370
COLLECTOR_UNTILT = 400
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

        self.current_drive_cmd = None   # Stores current drive twist
        self.hopper_cmd = None          # Stores current hoppy command
        self.collector_cmd = None       # Stores current collector spin command
        self.collector_tilt = None      # Stores current collector tilt command
        self.joy_received = False       

        #TODO: MAKE TOPIC NAMES PARAMETERS THAT CAN BE LOADED
        self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.hopper_pub = rospy.Publisher("hopper_control", Int16, queue_size = 10)
        self.collector_cmd_pub = rospy.Publisher("collector_spin_control", Int16, queue_size = 10)
        self.collector_tilt_pub = rospy.Publisher("collector_tilt_control", Int16, queue_size = 10)

        rospy.Subscriber("joy", Joy, self.joy_callback)

        atexit.register(self._exit_handler)

    def joy_callback(self, data):
        '''
        Joy topic callback
        '''
        ######
        # Build Twist message
        ######
        twister = Twist()
        # TANK DRIVE
        twister.linear.x = MAX_TANK_SPEED * data.axes[JOY_TANK_L_AXIS]
        twister.angular.z = MAX_TANK_SPEED * data.axes[JOY_TANK_R_AXIS]
        # Real Robot Drive
        #twister.angular.z = MAX_ANGULAR_SPEED * data.axes[JOY_ANGULAR_AXIS]
        #twister.linear.x = MAX_LINEAR_SPEED * data.axes[JOY_LINEAR_AXIS]
        self.current_drive_cmd = twister
        ###########################
        # Get Hopper command
        ##########################
        hopper_cmd = Int16()
        # get current button state (4 possible states)
        dump = True if data.buttons[JOY_DUMP_BTTN] == 1 else False
        undump = True if data.buttons[JOY_UNDUMP_BTTN] == 1 else False
        # determine what to do based on state
        if dump and not undump:
            hopper_cmd.data = HOPPER_DUMP
        elif not dump and undump:
            hopper_cmd.data = HOPPER_UNDUMP
        else:
            hopper_cmd.data = HOPPER_STOP 
        self.hopper_cmd = hopper_cmd
        ###########################
        # Get Conveyor Spin command
        ##########################
        collector_cmd = Int16()
        spin = True if data.buttons[JOY_CSPIN_BTTN] == 1 else False
        rspin = True if data.buttons[JOY_CRSPIN_BTTN] == 1 else False
        if spin and not rspin:
            collector_cmd.data = COLLECTOR_SPIN
        elif not spin and rspin:
            collector_cmd.data = COLLECTOR_RSPIN
        else:
            collector_cmd.data = COLLECTOR_STOP
        self.collector_cmd = collector_cmd
        ###########################
        # Get Conveyor tilt command
        ##########################
        tilt_cmd = Int16()
        tilt = True if data.buttons[JOY_CTILT_BTTN] == 1 else False 
        untilt = True if data.buttons[JOY_CUTILT_BTTN] == 1 else False
        if tilt and not untilt:
            tilt_cmd.data = COLLECTOR_TILT
        elif not tilt and untilt:
            tilt_cmd.data = COLLECTOR_UNTILT
        else:
            tilt_cmd.data = COLLECTOR_TSTOP
        self.collector_tilt = tilt_cmd

        self.joy_received = True

    def run(self):
        '''
        This function is the processing function for this module.
        '''
        rospy.wait_for_message("joy", Joy)  # Wait for messege on joy topic
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ###########################
            # Things that always get published every iteration
            self.drive_pub.publish(self.current_drive_cmd)
            self.hopper_pub.publish(self.hopper_cmd)
            self.collector_cmd_pub.publish(self.collector_cmd)
            self.collector_tilt_pub.publish(self.collector_tilt)
            if not self.joy_received:
                continue
            ###########################
            # Things that only get published upon receiving a new joy message

            rate.sleep()

    def _exit_handler(self):
        '''
        This function is called on exit 
        '''
        exit()

if __name__ == "__main__":
    controller = Joystick_Controller()
    controller.run()
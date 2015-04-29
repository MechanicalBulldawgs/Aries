#!/usr/bin/python

"""This node interfaces with an Arduino to control motors """

import rospy, serial, atexit
from threading import Lock
from collections import deque
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

###############
# Constants (TODO: parameter files)
HOPPER_CONTROL = 2 # PWM channel for hopper control
CONVEYOR_SPIN = 3  # PWM channel for conveyor spin motor
CONVEYOR_TILT = 4  # PWM channel for conveyor tilt motor
###############

class motor_director(object):
    
    def __init__(self):
        
        rospy.init_node('motor_director')
        self.cmd_queue = deque() # python's 'deck' data structure.  Used to store queue of commands received from callbacks to be handled.
        self.queue_lock = Lock()
        self.prev_cmds = {"HOPPER":None, "LINEARX":None, "ANGULARZ":None, "CONVEYOR_SPIN":None, "CONVEYOR_TILT":None}

        """Attempt to get parameters from the ROS server and use them to initialize the list 
            of touch sensors and the connection to the Arduino"""

        port = "/dev/ttyUSB0"#rospy.get_param('ports/arduino', '/dev/ttyACM0')
        print("Connecting to Arduino on port: " + str(port))
        self.arduino = serial.Serial(port, 9600, timeout = 1)
        print("Connected to Arduino on port: " + str(port))

        # Load topic names
        drive_topic               = rospy.get_param("topics/drive_cmds", "cmd_vel")
        hopper_cmds_topic         = rospy.get_param("topics/hopper_cmds", "hopper_control")
        collector_spin_cmds_topic = rospy.get_param("topics/collector_spin_cmds", "collector_spin_control")
        collector_tilt_cmds_topic = rospy.get_param("topics/collector_tilt_cmds", "collector_tilt_control")
        # Setup ROS subscribers
        rospy.Subscriber(drive_topic, Twist, self.cmd_vel_cb)
        rospy.Subscriber(hopper_cmds_topic, Int16, self.dump_callback)
        rospy.Subscriber(collector_spin_cmds_topic, Int16, self.collector_spin_callback)
        rospy.Subscriber(collector_tilt_cmds_topic, Int16, self.collector_tilt_callback)

        atexit.register(self._exit_handler)

    def run(self):
        '''
        This function processes cmds in cmd queue
        '''
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            with self.queue_lock:
                if len(self.cmd_queue) > 0:
                    cmd = self.cmd_queue.pop()
                    print("Sending: " + str(cmd))
                    self.arduino.write(str(cmd))
            rate.sleep()

    def cmd_vel_cb(self, data):
        '''
        Command velocity callback.  
        '''
        linear_cmd = "X:" + str(data.linear.x) + "\n"
        angular_cmd = "Z:" + str(data.angular.z) + "\n"
        with self.queue_lock:
            if self.prev_cmds["LINEARX"] != data.linear.x:
                self.prev_cmds["LINEARX"] = data.linear.x
                self.cmd_queue.appendleft(linear_cmd)
            if self.prev_cmds["ANGULARZ"] != data.angular.z:
                self.prev_cmds["ANGULARZ"] = data.angular.z
                self.cmd_queue.appendleft(angular_cmd)

    def dump_callback(self, data):
        cmd = str(HOPPER_CONTROL) + ":" + str(data.data) + "\n"
        with self.queue_lock:
            if self.prev_cmds["HOPPER"] != data.data:
                self.prev_cmds["HOPPER"] = data.data
                self.cmd_queue.appendleft(cmd)

    def collector_spin_callback(self, data):
        cmd = str(CONVEYOR_SPIN) + ":" + str(data.data) + "\n"
        with self.queue_lock:
            if self.prev_cmds["CONVEYOR_SPIN"] != data.data:
                self.prev_cmds["CONVEYOR_SPIN"] = data.data
                self.cmd_queue.appendleft(cmd)

    def collector_tilt_callback(self, data):
        cmd = str(CONVEYOR_TILT) + ":" + str(data.data) + "\n"
        with self.queue_lock:
            if self.prev_cmds["CONVEYOR_TILT"] != data.data:
                self.prev_cmds["CONVEYOR_TILT"] = data.data
                self.cmd_queue.appendleft(cmd)

    
    def _exit_handler(self):
        self.arduino.close()

if __name__ == "__main__":
    try:
        ser = motor_director()
    except rospy.ROSInterruptException as er:
        rospy.logerr(str(er))
    else:
        ser.run()
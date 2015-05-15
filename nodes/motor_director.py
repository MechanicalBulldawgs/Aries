#!/usr/bin/python

"""This node interfaces with an Arduino to control motors """

import rospy, serial, atexit, signal
from mbedrpc import *
from threading import Lock
from collections import deque
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

# Everything is -128 to 127

class motor_director(object):
    
    def __init__(self):
        
        rospy.init_node('motor_director')
        
        """Attempt to get parameters from the ROS server and use them to initialize the list 
            of touch sensors and the connection to the Arduino"""
        baudrate = rospy.get_param("baudrates/mbed", 115200)
        port = rospy.get_param('ports/mbed', '/dev/ttyACM0')
        self.mbed = SerialRPC(port, baudrate)
        print("Connected to mbed on port: " + str(port))

        # mbed variables (motor control)
        self.linear_velocity = RPCVariable(self.mbed, "LinearVelocity") # left
        self.angular_velocity = RPCVariable(self.mbed, "Angle") # right
        self.hopper = RPCVariable(self.mbed, "Hopper")
        self.collector_spin = RPCVariable(self.mbed, "Collector")
        self.collector_tilt = RPCVariable(self.mbed, "CollectorAngle")

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

        signal.signal(signal.SIGINT, self._signal_handler)
        atexit.register(self._exit_handler)

    def run(self):
        '''
        This function processes cmds in cmd queue
        '''
        rospy.spin()

    def cmd_vel_cb(self, data):
        '''
        Command velocity callback.  
        '''
        # TODO: check range of values between -128, 127
        print("Writing LV: " + str(int(data.linear.x)))
        self.linear_velocity.write(int(data.linear.x))
        print("Writing AZ: " + str(int(data.angular.z)))
        self.angular_velocity.write(int(data.angular.z))

    def dump_callback(self, data):
        print("Writing Hopper: " + str(int(data.data)))
        self.hopper.write(int(data.data))

    def collector_spin_callback(self, data):
        print("Writing Collector Spin: " + str(int(data.data)))
        self.collector_spin.write(int(data.data))

    def collector_tilt_callback(self, data):
        print("Writing Collector Tilt: " + str(int(data.data)))
        self.collector_tilt.write(int(data.data))
    
    def _exit_handler(self):
        # TODO: mbed serial close()?
        exit()

    def _signal_handler(self, signal, frame):
        exit()

if __name__ == "__main__":
    try:
        ser = motor_director()
    except rospy.ROSInterruptException as er:
        rospy.logerr(str(er))
    else:
        ser.run()
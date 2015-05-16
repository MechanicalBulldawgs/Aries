#!/usr/bin/python

"""This node interfaces with an Arduino to control motors """

import rospy, serial, atexit, signal
from mbedrpc import *
from threading import Lock
from collections import deque
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, UInt16
from byteclass import *

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
        self.mbed_twist = RPCVariable(self.mbed, "Twist")
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
        # Convert twist commands to something Ryan's mbed and read
        print("Writing LV: " + str(int(data.linear.x)))
        print("Writing AZ: " + str(int(data.angular.z)))
        msg = int(self._twist_to_ryan(data))
        # bytething = Byte(msg)
        # print("Test: " + str(self.test.read()))
        # print("MSG: " + str(bytething.binary()))
        self.mbed_twist.write(msg)
        # print("Test: " + str(self.test.read()))

    def _twist_to_ryan(self, twist):
        '''
        Given twist message, convert to single byte for ryan's mbed
        '''
        x = int(twist.linear.x)
        y = int(twist.angular.z)
        out=int('0x00',16);
        #test for negative
        if x<0:
            #make positve 
            temp = (-1)*x
            #set the 4th bit
            temp = temp | (1<<3)
            #shift to most significant nibble
            out = temp<<4
        else:
            #shift to mostsegnificant nibble
            out = x<<4

        if y<0:
            #make positve
            temp = (-1)*y
            #set the 4th bit
            temp = temp | (1<<3)
            #place in least signifcant nibble
            out= out|temp
        else:
            #place into least signifcant nibble
            out=out|y
        return out

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
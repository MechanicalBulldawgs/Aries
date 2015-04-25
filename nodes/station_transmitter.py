#!/usr/bin/env python
import socket, rospy, cPickle
import roslib; roslib.load_manifest("aries")

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int8

'''
This module is registered as a ROS node and transmits commands from the control station over a UDP socket to the robot.
 ** See control station diagram for more information **
'''
################################
# Constants
BUFFER_SIZE = 4096
################################

class Station_Transmitter(object):

    def __init__(self):
        '''
        Station Transmitter constructor
        '''
        rospy.init_node("station_transmitter")
        self.current_mode = 1
        ################################################
        ####### Load parameters from param files #######
        self.ROBOT_IP = rospy.get_param("control_station_comms/robot_ip", None)
        self.CONTROL_LINE_PORT  = int(rospy.get_param("control_station_comms/control_line_port", None))
        self.DATA_LINE_PORT     = int(rospy.get_param("control_station_comms/data_line_port", None))
        self.STATUS_RETURN_PORT = int(rospy.get_param("control_station_comms/status_return_port", None))

        # Load modes
        self.modes_by_val = {}
        modes = rospy.get_param("control_station_comms/control_modes")
        mode_vals = rospy.get_param("control_station_comms/mode_value")

        for i in xrange(0, len(modes)):
            try:
                value = int(mode_vals[i])
                name = modes[i]
            except:
                print("Failed to load control modes from parameter server.  Check parameter file.")
                exit()
            else:
                self.modes_by_val[value] = name
        print("Loaded modes: " + str(self.modes_by_val))

        ################################################
        #######   Setup comms with robot   #############
        self.status_ret_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_ret_sock.bind(("", self.STATUS_RETURN_PORT))

        self.data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        rospy.loginfo("Control station relaying messages to robot at " + str(self.ROBOT_IP)) # Transmits messages to robot
        # todo: announce mode ports with rospy.loginfo...

        ################################################
        #######    Subscribe to command topics   #######
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rospy.Subscriber("operation_mode", Int8, self.mode_callback)

    def run(self):
        '''
        '''
        rospy.spin()

    def joy_callback(self, data):
        '''
        This function gets called when message is received from joy topic
        '''
        # Pickle up message
        data_pickle = cPickle.dumps(data)
        # Send pickle if in correct mode
        if self.modes_by_val[self.current_mode] == "joystick": 
            self.data_sock.sendto(data_pickle, (self.ROBOT_IP, self.DATA_LINE_PORT))

    def mode_callback(self, data):
        '''
        '''
        # Get mode value
        self.current_mode = data.data
        # Send mode over socket to robot
        self.control_sock.sendto(str(self.current_mode), (self.ROBOT_IP, self.CONTROL_LINE_PORT))




if __name__ == "__main__":
    station = Station_Transmitter()
    station.run()
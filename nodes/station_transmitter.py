#!/usr/bin/env python
import socket, rospy, cPickle
import roslib; roslib.load_manifest("aries")

from sensor_msgs.msg import Joy

'''
This module is registered as a ROS node and transmits commands from the control station over a UDP socket to the robot.
 ** See control station diagram for more information **
'''
################################
# Constants
BUFFER_SIZE = 4096
JOYSTICK_MODE_NAME = rospy.get_param("control_station_comms/joystick_mode_name", "joystick")  
AUTONOMOUS_MODE_NAME = rospy.get_param("control_station_comms/autonomous_mode_name", "autonomous")
SUPERVISORY_MODE_NAME =  rospy.get_param("control_station_comms/supervisory_mode_name", "supervisory")
################################

class Station_Transmitter(object):

    def __init__(self):
        '''
        Station Transmitter constructor
        '''
        rospy.init_node("station_transmitter")

        ################################################
        ####### Load parameters from param files #######
        self.ROBOT_IP = rospy.get_param("control_station_comms/robot_ip", None)
        self.CONTROL_TRANS_PORT = rospy.get_param("control_station_comms/control_trans_port", None)
        self.ROBOT_TRANS_PORT = rospy.get_param("control_station_comms/robot_trans_port", None)
        # Load mode ports
        self.control_mode_ports = {}
        modes = rospy.get_param("control_station_comms/control_modes")
        for mode in modes:
            try:
                port = int(rospy.get_param("control_station_comms/" + str(mode) + "_port", -1))
                if port == -1: raise Exception()
            except:
                print("Failed to load control port for: " + str(mode))
            else:
                self.control_mode_ports[mode] = port
        print(self.control_mode_ports)

        ################################################
        #######   Setup comms with robot   #############
        self.robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.loginfo("Control station relaying messages to robot at " + str(self.ROBOT_IP)) # Transmits messages to robot
        # todo: announce mode ports with rospy.loginfo...
        ################################################
        #######    Subscribe to command topics   #######
        rospy.Subscriber("joy", Joy, self.joy_callback)

    def run(self):
        '''
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            mode = raw_input("Mode: ")
            try:
                int(mode)  # just check to make sure value is int
            except:
                print("Bad input")
            else:
                self.robot_sock.sendto(mode, (self.ROBOT_IP, self.CONTROL_TRANS_PORT))
            rate.sleep()

    def joy_callback(self, data):
        '''
        This function gets called when message is received from joy topic
        '''
        # Pickle up message
        data_pickle = cPickle.dumps(data)
        # Send pickle
        self.robot_sock.sendto(data_pickle, (self.ROBOT_IP, self.control_mode_ports[JOYSTICK_MODE_NAME]))





if __name__ == "__main__":
    station = Station_Transmitter()
    station.run()
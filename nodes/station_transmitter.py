#!/usr/bin/env python
import socket, rospy, cPickle
import roslib; roslib.load_manifest("aries")

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int8, Float32
from aries.msg import DurationCmd

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
        self.current_mode = None
        ################################################
        ####### Load parameters from param files #######
        global BUFFER_SIZE
        BUFFER_SIZE = int(rospy.get_param("control_station_comms/buffer_size"))
        self.ROBOT_IP = rospy.get_param("control_station_comms/robot_ip", None)
        self.CONTROL_LINE_PORT  = int(rospy.get_param("control_station_comms/control_line_port", -1))
        self.DATA_LINE_PORT     = int(rospy.get_param("control_station_comms/data_line_port", -1))
        self.STATUS_RETURN_PORT = int(rospy.get_param("control_station_comms/status_return_port", -1))

        # Load topic names
        joystick_topic = rospy.get_param("topics/joystick", "joy")
        op_mode_topic = rospy.get_param("topics/op_mode", "operation_mode")
        duration_cmds_topic = rospy.get_param("topics/duration_cmds", "duration_cmds")
        pivot_cmds_topic = rospy.get_param("topics/lidar_pivot_target_angles", "lidar_lidar_pivot_target_angles")

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
        rospy.Subscriber(joystick_topic, Joy, self.joy_callback)
        rospy.Subscriber(duration_cmds_topic, DurationCmd, self.duration_cmd_callback)
        rospy.Subscriber(op_mode_topic, String, self.mode_callback)
        rospy.Subscriber(pivot_cmds_topic, Float32, self.lidar_pivot_callback)

    def run(self):
        '''
        '''
        rospy.spin()

    def lidar_pivot_callback(self, data):
        '''
        This function gets called when a message is received from the lidar pivot cmds topic 
        '''
        data_pickle = cPickle.dumps(data)
        if self.current_mode == "lidar_pivot":
            self.data_sock.sendto(data_pickle, (self.ROBOT_IP, self.DATA_LINE_PORT))

    def joy_callback(self, data):
        '''
        This function gets called when message is received from joy topic
        '''
        # Pickle up message
        data_pickle = cPickle.dumps(data)
        # Send pickle if in correct mode
        if self.current_mode == "joystick": 
            self.data_sock.sendto(data_pickle, (self.ROBOT_IP, self.DATA_LINE_PORT))

    def duration_cmd_callback(self, data):
        '''
        '''
        data_pickle = cPickle.dumps(data)
        if self.current_mode == "duration_teleop":
            self.data_sock.sendto(data_pickle, (self.ROBOT_IP, self.DATA_LINE_PORT))




    def mode_callback(self, data):
        '''
        '''
        # Get mode value
        self.current_mode = str(data.data)
        # Send mode over socket to robot
        self.control_sock.sendto(str(self.current_mode), (self.ROBOT_IP, self.CONTROL_LINE_PORT))




if __name__ == "__main__":
    station = Station_Transmitter()
    station.run()
#!/usr/bin/env python
import socket, atexit, signal, rospy
import roslib; roslib.load_manifest("aries")

from multiprocessing import Process, Value
from sensor_msgs.msg import Joy

'''
This module runs on the robot and receives messages over a UDP socket connection
    and translates it into ROS message commands.
** See control system diagram for more information **
'''

################################
# Constants
BUFFER_SIZE = 4096
JOYSTICK_MODE_NAME = rospy.get_param("control_station_comms/joystick_mode_name", "joystick")  
AUTONOMOUS_MODE_NAME = rospy.get_param("control_station_comms/autonomous_mode_name", "autonomous")
SUPERVISORY_MODE_NAME =  rospy.get_param("control_station_comms/supervisory_mode_name", "supervisory")
################################

class Station_Receiver(object):

    def __init__(self):
        '''
        Station Receiver constructor
        '''
        rospy.init_node("station_receiver")
        self.mode = JOYSTICK_MODE_NAME
        ################################################
        ####### Load parameters from param files #######
        self.STATION_IP = rospy.get_param("control_station_comms/station_ip", None)
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
        self.robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Receives messages from station
        self.robot_sock.bind(("", self.CONTROL_TRANS_PORT))

        ################################################
        #######   Setup process to handle control line   #############
        self.shared_ctrl_mode = Value("i", 0)
        self.ctrl_process = Process(target = self.ctrl_line, args = (self.shared_ctrl_mode,))
        self.ctrl_process.start()

        rospy.loginfo("Robot listening to station at: " + str(self.STATION_IP))
        # todo: announce mode ports with rospy.loginfo...
        atexit.register(self._exit_handler)

    def ctrl_line(self, shared_mode):
        '''
        '''
        # Check for data on ctrl line
        ctrl_data, addr = self.robot_sock.recvfrom(BUFFER_SIZE)
        try:
            shared_mode.value = int(ctrl_data)
        except:
            print("Bad Control Line Data")
        
    def run(self):
        '''
        '''
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            # Check for data on appropriate cmd line (handle potential errors gracefully)
            print(self.shared_ctrl_mode.value)
            rate.sleep()
            # Publich data from cmd line over appropriate topic
    
    def _exit_handler(self):
        '''
        This function is called at exit
        '''
        self.ctrl_process.terminate()
        self.ctrl_process.join()

    def _signal_handler(self, signal, frame):
        '''
        Called when ctr-c signal is received
        '''
        exit()

if __name__ == "__main__":
    receiver = Station_Receiver()
    receiver.run()
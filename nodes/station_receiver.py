#!/usr/bin/env python
import socket, atexit, signal, rospy, cPickle

from multiprocessing import Process, Value, Lock
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from aries.msg import DurationCmd

'''
This module runs on the robot and receives messages over a UDP socket connection
    and translates it into ROS message commands.
** See control system diagram for more information **
'''

################################
# Constants
BUFFER_SIZE = 4096
################################

class Station_Receiver(object):

    def __init__(self):
        '''
        Station Receiver constructor
        '''
        rospy.init_node("station_receiver")
        
        ################################################
        ####### Load parameters from param files #######
        self.CONTROL_LINE_PORT  = int(rospy.get_param("control_station_comms/control_line_port", -1))
        self.DATA_LINE_PORT     = int(rospy.get_param("control_station_comms/data_line_port", -1))
        self.STATUS_RETURN_PORT = int(rospy.get_param("control_station_comms/status_return_port", -1))
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

        # Load topic names
        op_mode_topic = rospy.get_param("topics/op_mode", "operation_mode")
        joystick_topic = rospy.get_param("topics/joystick", "joy")
        duration_cmds_topic = rospy.get_param("topics/duration_cmds", "duration_cmds")
        
        self.duration_cmds_pub = rospy.Publisher(duration_cmds_topic, DurationCmd, queue_size = 10)
        self.mode_pub = rospy.Publisher(op_mode_topic, String, queue_size = 10)
        self.joy_pub = rospy.Publisher(joystick_topic, Joy, queue_size = 10)
        ################################################
        #######   Setup comms with robot   #############
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Receives control messages from station
        self.control_sock.bind(("", self.CONTROL_LINE_PORT))

        self.data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.data_sock.bind(("", self.DATA_LINE_PORT))

        ##############################################################
        #######   Setup process to handle control line   #############
        self.shared_ctrl_mode = Value("i", 1)
        self.mode_lock = Lock()
        self.ctrl_process = Process(target = self.ctrl_line, args = (self.shared_ctrl_mode, self.mode_lock))
        self.ctrl_process.start()

        # todo: announce mode ports with rospy.loginfo...
        atexit.register(self._exit_handler)

    def ctrl_line(self, shared_mode, mode_lock):
        '''
        This function runs as background process.  Listens to control line for updates to cmd mode.
        '''
        # Check for data on ctrl line
        ctrl_data, addr = self.control_sock.recvfrom(BUFFER_SIZE)
        try:
            mode_lock.acquire()
            mode = int(ctrl_data)
            mode_lock.release()
        except:
            print("Bad Control Line Data")
        else:
            shared_mode.value = mode 
            print("New mode: " + str(mode))
            self.mode_pub.publish(self.modes_by_val[mode])
        
    def run(self):
        '''
        '''
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # Wait to receive data over data line
            data, addr = self.data_sock.recvfrom(BUFFER_SIZE)
            print("Current Mode: " + str(mode))
            # Safely get current mode
            self.mode_lock.acquire()
            current_mode = self.shared_ctrl_mode.value 
            self.mode_lock.release()
            # Unpickle data
            data = cPickle.loads(data)
            # Handle data appropriately
            if self.modes_by_val[current_mode] == "joystick":
                # Relay joy message
                self.joy_pub.publish(data)
            elif self.modes_by_val[current_mode] == "duration_teleop":
                # Relay d command
                self.duration_cmds_pub.publish(data)
            elif self.modes_by_val[current_mode] == "autonomous":
                pass

            rate.sleep()
    
    def _exit_handler(self):
        '''
        This function is called at exit
        '''
        try:
            self.ctrl_process.terminate()
        except:
            pass
        try:
            self.ctrl_process.join()
        except:
            pass

    def _signal_handler(self, signal, frame):
        '''
        Called when ctr-c signal is received
        '''
        exit()

if __name__ == "__main__":
    receiver = Station_Receiver()
    receiver.run()
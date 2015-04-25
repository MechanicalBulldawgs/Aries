#!/usr/bin/env python
import socket, rospy, cPickle, signal, atexit
import roslib; roslib.load_manifest("aries")

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int8

'''
This module provides an interface to select robot operation modes.
  Modes are transmitted over operation mode topic
'''

class Mode_Selector(object):

    def __init__(self):
        '''
        '''
        rospy.init_node("mode_selector")
        self.mode_pub = rospy.Publisher("operation_mode", Int8)

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

        atexit.register(self._exit_handler)
        signal.signal(signal.SIGINT, self._signal_handler)


    def run(self):
        '''
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            print("=== MODE SELECTION MENU ===")
            for key in self.modes_by_val:
                print(str(self.modes_by_val[key] + ": " + str(key)))
            print("Exit: q")
            mode = raw_input("Mode>> ")
            if mode == "q": exit()
            try:
                int(mode)  # just check to make sure value is int
            except:
                print("Bad input.")
            else:
                msg = Int8()
                msg.data = int(mode)
                self.mode_pub.publish(msg)
            rate.sleep()

    def _exit_handler(self):
        '''
        This function gets called on exit
        '''
        exit()

    def _signal_handler(self, signal, frame):
        exit()


if __name__ == "__main__":
    ui = Mode_Selector()
    ui.run()
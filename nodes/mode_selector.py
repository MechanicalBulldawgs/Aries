#!/usr/bin/env python
import socket, rospy, cPickle, signal, atexit
import roslib; roslib.load_manifest("aries")

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int8
from aries.msg import DurationCmd

'''
This module provides an interface to select robot operation modes.
  Modes are transmitted over operation mode topic

  TODO: MAKE THIS BETTER! -- PRETTY GROSS RIGHT NOW.
'''
DEFAULT_MODE = "joystick"

class Mode_Selector(object):

    def __init__(self):
        '''
        '''
        rospy.init_node("mode_selector")
        self.mode_pub = rospy.Publisher("operation_mode", Int8, queue_size = 10)

        # Load modes
        self.modes_by_val = {}
        self.vals_by_mode = {}
        self.current_mode = None
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
                self.vals_by_mode[name] = value
        print("Loaded modes: " + str(self.modes_by_val))

        cmds_topic = rospy.get_param("topics/duration_cmds", "duration_cmds")
        self.duration_cmds_pub = rospy.Publisher(cmds_topic, DurationCmd, queue_size = 10)


        atexit.register(self._exit_handler)
        signal.signal(signal.SIGINT, self._signal_handler)


    def run(self):
        '''
        '''
        rate = rospy.Rate(10)

        # publish default mode
        rospy.sleep(0.1)
        default_msg = Int8()
        default_msg.data = self.vals_by_mode[DEFAULT_MODE]
        self.mode_pub.publish(default_msg)

        while not rospy.is_shutdown():

            if self.current_mode == "duration_teleop":
                # duration teleoperation mode
                print("===== Command Menu: =====")
                print(" - forward <duration>\n - backward <duration>\n - left <duration>\n - right <duration>\n - collect <duration>\n - tilt-collector \n - dump \n - stop")
                print(" - Exit duration_teleop mode: cm")
                print("=========================")
                uinput = raw_input("Enter command: ")
                if uinput == "q": exit()

                if uinput == "cm": 
                    self.current_mode = DEFAULT_MODE
                    msg = Int8()
                    msg.data = self.vals_by_mode[DEFAULT_MODE]
                    self.mode_pub.publish(msg)
                    continue

                dcmd = DurationCmd()
                try:
                    uinput = uinput.split(" ")
                    cmd = uinput[0]
                    d = 0 if len(uinput) == 1 else float(uinput[1])
                    dcmd.cmd = cmd 
                    dcmd.duration = d 
                except:
                    print("poorly formatted input")
                else:
                    self.duration_cmds_pub.publish(dcmd)
            else:
                print("=== MODE SELECTION MENU ===")
                for key in self.modes_by_val:
                    print(str(self.modes_by_val[key] + ": " + str(key)))
                print("Exit: q")

                uinput = raw_input("Mode>> ")
                if uinput == "q": exit()
                
                try:
                    int(uinput)  # just check to make sure value is int
                    self.modes_by_val[int(uinput)]
                except:
                    print("Bad input.")
                else:
                    self.current_mode = self.modes_by_val[int(uinput)]
                    msg = Int8()
                    msg.data = int(uinput)
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
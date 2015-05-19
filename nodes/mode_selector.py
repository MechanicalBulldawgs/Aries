#!/usr/bin/env python
import socket, rospy, cPickle, signal, atexit, math
import roslib; roslib.load_manifest("aries")

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int8, Float32
from aries.msg import DurationCmd

'''
This module provides an interface to select robot operation modes.
  Modes are transmitted over operation mode topic

  TODO: MAKE THIS BETTER! -- PRETTY GROSS RIGHT NOW.
'''


class Mode_Selector(object):
    DEFAULT_MODE = "joystick"

    def __init__(self):
        '''
        '''
        rospy.init_node("mode_selector")
        self.mode_pub = rospy.Publisher("operation_mode", String, queue_size = 10)

        # Load modes
        self.current_mode = None
        modes = rospy.get_param("control_station_comms/control_modes")
        Mode_Selector.DEFAULT_MODE = rospy.get_param("control_station_comms/default_mode", "joystick")
        self.valid_modes = [mode for mode in modes]
        print("Loaded modes: " + str(self.valid_modes))

        # Load LIDAR Pivot stuff
        self.PIVOT_LAYDOWN = rospy.get_param("dynamixel_settings/laying_angle", 20)
        self.PIVOT_STAND = rospy.get_param("dyanmixel_settings/standing_angle", 0)
        self.PIVOT_MAX = rospy.get_param("dyanimxel_settings/top_limit", 25)
        self.PIVOT_MIN = rospy.get_param("dynamixel_settings/bottom_limit", -1)

        cmds_topic = rospy.get_param("topics/duration_cmds", "duration_cmds")
        self.duration_cmds_pub = rospy.Publisher(cmds_topic, DurationCmd, queue_size = 10)
        pivot_cmds_topic = rospy.get_param("topics/lidar_pivot_target_angles", "lidar_lidar_pivot_target_angles")
        self.lidar_pivot_pub = rospy.Publisher(pivot_cmds_topic, Float32, queue_size = 10)

        atexit.register(self._exit_handler)
        signal.signal(signal.SIGINT, self._signal_handler)


    def run(self):
        '''
        '''
        rate = rospy.Rate(10)
        # publish default mode
        rospy.sleep(0.1)
        default_msg = String()
        default_msg.data = Mode_Selector.DEFAULT_MODE
        self.mode_pub.publish(default_msg)

        while not rospy.is_shutdown():
            if self.current_mode == "duration_teleop":
                # duration teleoperation mode
                print("===== Command Menu: =====")
                print(" - forward <duration>\n - backward <duration>\n - left <duration>\n - right <duration>\n - collect <duration>\n - uncollect <duration>\n - tilt-collector \n - dump \n - undump \n - stop")
                print(" - take-dump \n - mine <duration> \n - arc-left <duration>\n - arc-left-rev <duration>\n - arc-right <duration>\n - arc-right-rev <duration>\n")
                print(" - Exit duration_teleop mode: cm")
                print("=========================")
                uinput = raw_input("Enter command: ")
                if uinput == "q": exit()

                if uinput == "cm": 
                    self.current_mode = Mode_Selector.DEFAULT_MODE
                    msg = String()
                    msg.data = Mode_Selector.DEFAULT_MODE
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
            elif self.current_mode == "lidar_pivot":
                # lidar pivot control
                print("===== Command Menu: =====")
                print(" - stand")
                print(" - lay-down")
                print(" - angle <target angle (in degrees)>")
                print(" - Exit lidar pivot control: cm")
                print("=========================")
                uinput = raw_input("Enter command: ")
                if uinput == "q": exit()
                if uinput == "cm":
                    self.current_mode = Mode_Selector.DEFAULT_MODE
                    msg = String()
                    msg.data = Mode_Selector.DEFAULT_MODE
                    self.mode_pub.publish(msg)
                    continue
                angle_cmd = Float32()
                uinput = uinput.split(" ")

                if uinput[0] == "stand":
                    angle_cmd.data = math.radians(0)
                    self.lidar_pivot_pub.publish(angle_cmd)
                elif uinput[0] == "lay-down":
                    angle_cmd.data = math.radians(20)
                    self.lidar_pivot_pub.publish(angle_cmd)
                elif uinput[0] == "angle":
                    try:
                        a = float(uinput[1])
                    except:
                        print("bad input")
                    else:
                        if a >= self.PIVOT_MIN and a <= self.PIVOT_MAX:
                            angle_cmd.data = math.radians(a)
                            self.lidar_pivot_pub.publish(angle_cmd)
                        else:
                            print("Angle out of range")
                else:
                    print("bad input")

            else:
                print("=== MODE SELECTION MENU ===")
                for mode in self.valid_modes:
                    print(" - " + str(mode))
                print("Exit: q")

                uinput = raw_input("Mode>> ")
                if uinput == "q": exit()
                if str(uinput) in self.valid_modes:
                    self.current_mode = str(uinput)
                    msg = String()
                    msg.data = str(uinput)
                    self.mode_pub.publish(msg)
                else:
                    print("Bad Input")
                    
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
#!/usr/bin/env python
import socket
import roslib; roslib.load_manifest("aries")

'''
This module runs on the robot and receives messages over a UDP socket connection
    and translates it into ROS message commands.
** See control system diagram for more information **
'''


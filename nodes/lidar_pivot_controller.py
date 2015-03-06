#!/usr/bin/env python

import rospy
from  lib_robotis import USB2Dynamixel_Device, Robotis_Servo
from math import *
from std_msgs.msg import Float32

'''
This module is responsible for interfacing with lidar pivot motor.
'''

class lidar_pivot_controller(object):

    def __init__(self):
        '''
        lidar pivot controller constructor
        '''
        self.angle = 0
        #Creates the ROS node.
        rospy.init_node("lidar_pivot_controller")

        #Servo Motor Setup
        self.dyn = USB2Dynamixel_Device(dev_name = "/dev/ttyUSB0", baudrate = 1000000)
        self.servo = Robotis_Servo(self.dyn, 1)

        #Inits the LIDAR Subscriber
        rospy.Subscriber("lidar_pivot_angle",Float32,self.angle_callback)

    def angle_callback(self,angle):
        self.angle = angle.data


    def run(self):
        #Runs while shut down message is not recieved.
        while not rospy.is_shutdown():

            #Sends command to move Dynamixel to absolute position. 
            self.servo.move_angle(radians(self.angle))

            rospy.sleep(0.5)    #Keeps ROS from crashing

if __name__ == "__main__":
    controller = lidar_pivot_controller()
    controller.run()
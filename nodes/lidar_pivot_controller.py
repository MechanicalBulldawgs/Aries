#!/usr/bin/env python

import roslib; roslib.load_manifest('aries')
import rospy
from  lib_robotis import USB2Dynamixel_Device, Robotis_Servo
from math import *
from std_msgs.msg import Float32
from aries.srv import LidarPivotAngle, LidarPivotAngleResponse, LidarPivotAngleRequest

'''
This module is responsible for interfacing with lidar pivot motor.
'''
INITIAL_ANGLE = 0   # Angle servo initializes to
DYNAMIXEL_ID = 1    # ID for dynamixel servo

class lidar_pivot_controller(object):

    def __init__(self):
        '''
        lidar pivot controller constructor
        '''
        # Creates the ROS node.
        rospy.init_node("lidar_pivot_controller")

        # Load relevant parameters from ROS parameter server
        dyn_port = rospy.get_param("ports/dynamixel", "/dev/ttyUSB0")
        dyn_baud = rospy.get_param("baudrates/dynamixel_baud", 1000000)

        self.target_angle = INITIAL_ANGLE       # Target angle in radians
        self.move_request = False

        # Servo Motor Setup
        self.dyn = USB2Dynamixel_Device(dev_name = dyn_port, baudrate = dyn_baud)
        self.servo = Robotis_Servo(self.dyn, DYNAMIXEL_ID)

        LIDAR_PIVOT_CONTROL = rospy.get_param("topics/lidar_pivot_control")
        GET_ANGLE_SERVICE = rospy.get_param("services/get_angle_service")
        # Inits the LIDAR Subscriber
        rospy.Subscriber(LIDAR_PIVOT_CONTROL, Float32, self.angle_callback)

        # Initialize service that gets the current angle of the lidar
        self.get_angle_service = rospy.Service(GET_ANGLE_SERVICE, LidarPivotAngle, self.handle_get_lidar_pivot_position)

        # Send servo to default position
        self.servo.move_angle(self.target_angle)
        self.current_angle = self.servo.read_angle()

    def angle_callback(self, angle):
        '''
        lidar_pivot_angle topic callback function
        '''
        self.target_angle = angle.data
        self.move_request = True


    def handle_get_lidar_pivot_position(self, req):
        '''
        get_lidar_pivot_position Service handler.
        '''
        response = LidarPivotAngleResponse(self.servo.read_angle())
        return response

    def run(self):
        # #Runs while shut down message is not recieved.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Sends command to move Dynamixel to absolute position.
            if self.move_request: 
                self.move_request = False
                self.servo.move_angle(self.target_angle)
                
            rate.sleep()    # Keeps ROS from crashing

if __name__ == "__main__":
    controller = lidar_pivot_controller()
    controller.run()
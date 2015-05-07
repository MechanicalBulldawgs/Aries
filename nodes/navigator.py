#!/usr/bin/env python

import rospy, signal, atexit, math
from geometry_msgs.msg import Twist, Point, Pose
from tf.transformations import euler_from_quaternion

'''
This module is responsible for sending Twist commands to robot.
Input: Waypoints, Map
Output: Twist commands
'''
######################################
# Global constants
GOAL_FORCE_CONST = 1.0  # magnitude used when calculating goal force
######################################
# Load global topic names from ros params
######################################
DRIVE_TOPIC = rospy.get_param("topics/drive_cmds", "cmd_vel")
GOAL_TOPIC = rospy.get_param("topics/navigation_goals", "nav_goal")
ROBOPOSE_TOPIC = rospy.get_param("topics/localization_pose", "beacon_localization_pose")

class PFieldNavigator(object):

    def __init__(self):
        '''
        Potential field navigator constructor.
        '''
        rospy.init_node("pfield_navigator")
        self.robot_pose = Pose()
        self.current_goal = Point()
       
        
        ######################################
        # Setup ROS publishers
        ######################################
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, Twist, queue_size = 10)

        ######################################
        # Setup ROS subscibers
        ######################################
        rospy.Subscriber(ROBOPOSE_TOPIC, Pose, self.robot_pose_callback)
        rospy.Subscriber(GOAL_TOPIC, Point, self.nav_goal_callback)

    def nav_goal_callback(self, data):
        '''
        Callback for navigation goals.
        '''
        self.current_goal = data


    def robot_pose_callback(self, data):
        '''
        Callback for robot localization pose.
        '''
        self.robot_pose = data

    def run(self):
        '''
        '''
        rate = rospy.Rate(10)
        # hold up for some messages
        rospy.wait_for_message(ROBOPOSE_TOPIC , Pose)
        rospy.wait_for_message(GOAL_TOPIC, Point)
        # work hard doing good stuff
        while not rospy.is_shutdown():
            # grab current goal and pose information
            nav_goal = self.current_goal
            robot_pose = self.robot_pose 
            # Calculate goal force
            goal_force = self.calc_goal_force(nav_goal, robot_pose)
            print(str(goal_force))
            # Calculate force from obstacles

            # Get final drive vector (goal, obstacle forces)

            # Calculate twist message from drive vector

            rate.sleep()

    def calc_goal_force(self, nav_goal, robot_pose):
        '''
        given a goal point and a robot pose, calculate and return x and y components of goal force
        '''
        # get angle to goal
        angle_to_goal = math.atan2(nav_goal.y - robot_pose.position.y, nav_goal.x - robot_pose.position.x)
        # convert robot pose orientation from quats to eulers
        robot_orient = euler_from_quaternion([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w]) 
        # get force angle
        force_angle = self.wrap_angle(angle_to_goal - robot_orient[2])
        # math the components
        s_x = GOAL_FORCE_CONST * math.cos(force_angle)
        s_y = GOAL_FORCE_CONST * math.sin(force_angle)
        return (s_x, s_y)


    def wrap_angle(self, angle):
        #This function will take any angle and wrap it into the range [-pi, pi]
        while angle >= math.pi:
            angle = angle - 2 * math.pi
            
        while angle <= -math.pi:
            angle = angle + 2 * math.pi
        return angle

if __name__ == "__main__":
    navigator = PFieldNavigator()
    navigator.run()

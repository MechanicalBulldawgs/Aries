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
ANGULAR_SPEED = 1.0
LINEAR_SPEED = 1.0
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
            print("==============================")
            print(" **  Goal: \n" + str(nav_goal))
            print(" ** Position: \n" + str(robot_pose.position))
            # Calculate goal force
            goal_force = self.calc_goal_force(nav_goal, robot_pose)
            print("Goal force: " + str(goal_force))
            # Calculate force from obstacles
            # TODO
            # Get final drive vector (goal, obstacle forces)
            # TODO
            # Calculate twist message from drive vector
            drive_cmd = self.drive_from_force(goal_force, robot_pose)
            #self.drive_pub.publish(drive_cmd)
            rate.sleep()

    def drive_from_force(self, force, robot_pose):
        '''
        Given a force vector, generate Twist message 
        '''
        cmd = Twist()
        max_angle = math.pi
        spin_thresh = math.pi 
         # convert quat orientation to eulers
        robot_orient = euler_from_quaternion([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w]) 
        # Get force angle (in global space)
        force_angle = math.atan2(force[1], force[0])
        # put force angle in robot space
        force_angle = -1 * (force_angle - (math.pi / 2.0))
        # get force magnitude
        force_mag = math.hypot(force[0], force[1])
        # get difference to robot's current yaw
        angle_diff = self.wrap_angle(force_angle - robot_orient[2])
        print("Robot Yaw: " + str(math.degrees(robot_orient[2])))
        print("Force angle: " + str(math.degrees(force_angle)))
        print("Force Magnitude: " + str(force_mag))
        print("Angle diff: " + str(math.degrees(angle_diff)))
        if force_mag == 0: return cmd
        ang_vel = (angle_diff / max_angle) * ANGULAR_SPEED
        lin_vel = 0 if abs(angle_diff) >= spin_thresh else force_mag

        print("Ang vel: " + str(ang_vel))
        print("Lin Vel: " + str(lin_vel))
        cmd.angular.z = ang_vel
        cmd.linear.x = lin_vel

        return cmd

    def calc_goal_force(self, nav_goal, robot_pose):
        '''
        given a goal point and a robot pose, calculate and return x and y components of goal force
        '''
        GOAL_THRESH = 0.05 # radius around goal that it's okay to stop in 
        FIELD_SPREAD = 10.0 # radius around goal where pfield is scaled
        ALPHA = 1.0
        # get distance between goal and robot
        dist = math.sqrt((nav_goal.x - robot_pose.position.x)**2 + (nav_goal.y - robot_pose.position.y)**2)
        # get angle to goal
        angle_to_goal = math.atan2(nav_goal.y - robot_pose.position.y, nav_goal.x - robot_pose.position.x)
        # get force angle
        force_angle = self.wrap_angle(angle_to_goal)
        # math the components
        if dist < GOAL_THRESH:
            d_x = 0
            d_y = 0
        elif GOAL_THRESH <= dist <= FIELD_SPREAD + GOAL_THRESH:
            d_x = ALPHA * (dist - GOAL_THRESH) * math.cos(force_angle)
            d_y = ALPHA * (dist - GOAL_THRESH) * math.sin(force_angle)
        else: #dist > (FIELD_SPREAD + GOAL_THRESH)
            d_x = ALPHA * FIELD_SPREAD * math.cos(force_angle)
            d_y = ALPHA * FIELD_SPREAD * math.sin(force_angle)

        return (d_x, d_y)

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

#!/usr/bin/env python

import time, rospy
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Bool, String

#state constants
stateStart = 0
stateNavMining = 1
stateMine = 2
stateNavBin = 3
stateDump = 4


class Autonomy_State_Machine(object):

	def __init__(self):
		'''
		'''
		rospy.init_node("autonomy_node")
		
		self.hopper_state = None
		self.autonomous = False
		
		self.startComplete = False
		self.navBinComplete = False
		self.miningComplete = False
		self.navMiningComplete = False
		
		self.transitionTime = 0
		self.reached_goal = False
		self.robot_pose = Pose()
		self.navMiningFlag = True
		self.waypointsMined = 0
		self.mineStart = False
		self.navBinFlag = True
		self.dumpStatePublishedFlag = True
		self.dumpTime = 0
		
		#Load parameters
		mining_waypoints = rospy.get_param("waypoints/mining", "mining")
		mining_area = rospy.get_param("waypoints/mining_area", "mining_area")
		bin_waypoint = rospy.get_param("waypoints/bin", "bin")
		COLLECTOR_SPIN = int(rospy.get_param("collector_settings/spin_signal"))
		COLLECTOR_STOP = int(rospy.get_param("collector_settings/spin_stop_signal"))
		
		# Load topics
		hopper_state_topic = rospy.get_param("topics/hopper_state", "hopper_state")
		dump_cmds_topic = rospy.get_param("topics/dump_cmds", "dump_cmds")
		REACHED_GOAL_TOPIC = rospy.get_param("topics/reached_goal", "reached_goal")
		GOAL_TOPIC = rospy.get_param("topics/navigation_goals", "nav_goal")
		ROBOPOSE_TOPIC = rospy.get_param("topics/localization_pose", "beacon_localization_pose")
		OP_MOD_TOPIC = rospy.get_param("topics/op_mode")
		

		
		
		# Setup subscriptions
		rospy.Subscriber(hopper_state_topic, String, self.hopper_state_callback)
		rospy.Subscriber(REACHED_GOAL_TOPIC, Bool, self.reached_goal_callback)
		rospy.Subscriber(ROBOPOSE_TOPIC, PoseStamped, self.robot_pose_callback)
		rospy.Subscriber(OP_MOD_TOPIC, String, self.op_mode_callback)
		
		# Setup publishers
		self.dump_pub = rospy.Publisher(dump_cmds_topic, String, queue_size = 10)
		self.goal_pub = rospy.Publisher(GOAL_TOPIC, Point, queue_size = 10)
		
		self.current_state = stateStart

	def run(self):
		'''
		STATE MACHINE!
		'''
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			if self.stateMachineOff():
				self.current_state = stateStart
				rate.sleep()
				continue

			if self.current_state == stateStart:
				self.start_state()
			elif self.current_state == stateNavMining:
				self.nav_mining_state()
			elif self.current_state == stateMine:
				self.mine_state()
			elif self.current_state == stateNavBin:
				self.nav_bin_state()
			elif self.current_state == stateDump:
				self.dump_state()
			else:
				#something went wrong, should never be meet
				#may want some other form of error handling
				self.current_state = stateStart
			
			self.transition()
			rate.sleep()

	def start_state(self):
		'''
		This function runs the start state.
		This state is responsible for initizializing 
		everything and verifying initilizations.
		Also, Find the beacon for the first time, the very first time.
		'''
		pass

	def nav_mining_state(self):
		'''
		This state is responsible for navigating to the mining area.
		'''
		if mining_area[1] <= self.robot_pose.position.y:
			self.navMiningComplete = True
		elif self.navMiningFlag:
				self.navMiningFlag = False
				goal_point = Point()
				goal_point.x = mining_waypoints[0][0]
				goal_point.y = mining_waypoints[0][1]
				goal_point.z = 0
			

	def mine_state(self):
		'''
		This state is responsible for mining dirt.  The robot will mine X number of waypoints in this state.
		'''
		numWaypoints = 3
		
		if not self.mineStart:
			collect_cmd = Int16()
			collect_cmd.data = COLLECTOR_SPIN
			self.collector_spin_pub.publish(collect_cmd)
			self.mineStart = True
		
		if self.reached_goal:
			if self.waypointsMined < numWaypoints:
				goal_point = Point()
				goal_point.x = mining_waypoints[self.waypointsMined][0]
				goal_point.y = mining_waypoints[self.waypointsMined][1]
				goal_point.z = 0
				self.waypointsMined += 1
				self.goal_pub.publish(goal_point)
			else:
				self.miningComplete = True
				self.mineStart = False
				collect_cmd = Int16()
				collect_cmd.data = COLLECTOR_STOP
				self.collector_spin_pub.publish(collect_cmd)

	def nav_bin_state(self):
		'''
		This state is responsible for navigating to the collection bin (backwards).
		Also ensuring the robot is close enough to collection bin for dumping.
		'''
		if self.navBinFlag:
			goal_point = Point()
			goal_point.x = bin_waypoint[self.waypointsMined][0]
			goal_point.y = bin_waypoint[self.waypointsMined][1]
			goal_point.z = bin_waypoint[self.waypointsMined][2]
			self.goal_pub.publish(goal_point)
			self.navBinFlag = False
		elif self.reached_goal:
			self.navBinComplete = True
			self.navBinFlag = True

	def dump_state(self):
		'''
		This state is responsible for dumping dirt into the collection bin.
		Ensuring the dirt has completely dumped.
		'''
		if dumpStatePublishedFlag:
			dump_cmd = String()
			dump_cmd.data = "DUMP"
			self.dump_pub.publish(dump_cmd)
			self.dumpStatePublishedFlag = False
			self.dumpTime = time.time()


	def transition(self):
		'''
		Transition function:
		STATE: START
		- Trans Condition: initilizations verified -> NAV MINING STATE 
		STATE: NAV MINING
		- Trans Condition: Reach mining area -> MINING STATE 
		STATE: MINE
		- Trans Condition: Mine X number of waypoints in mining area -> NAV BIN STATE
		STATE: NAV BIN
		- Trans Condition: Localization says we're at bin -> DUMP STATE 
		STATE: DUMP 
		- Trans Condition: Dump complete -> NAV MINING STATE 
		'''
		if (time.time() < (self.transitionTime - 1)):
			#we do not want to accidentally skip a state because of something not updating quickly enough
			return
		
		if self.current_state == stateStart:
			if self.startComplete:
				self.current_state = stateNavMining
				self.startComplete = False
		elif self.current_state == stateNavMining:
			if self.navMiningComplete:
				self.current_state = stateMine
				self.navMiningComplete = False
				self.navMiningFlag = True
		elif self.current_state == stateMine:
			if self.miningComplete:
				self.current_state = stateNavBin
				self.waypointsMined = 0
				self.miningComplete = False
		elif self.current_state == stateNavBin:
			if self.navBinComplete:
				self.current_state = stateDump
				self.navBinComplete = False
		elif self.current_state == stateDump:
			#when leaving the dump state make self.dumpStatePublishedFlag = False
			if self.hopper_state == "RESTING":
				if ((self.dumpTime + 10) == time.time()):
					self.current_state = stateNavMining
					self.dumpStatePublishedFlag = False

	def stateMachineOff(self):
		return  not self.autonomous
			
	def hopper_state_callback(self, state):
		self.hopper_state = state.data
		
	def reached_goal_callback(self, data):
		self.reached_goal = data.data
		
	def robot_pose_callback(self, data):
		self.robot_pose = self.transform_pose(data.pose)

	def op_mode_callback(self, data):
		self.autonomous = (data.data=='autonomous')

if __name__ == "__main__":
	try:
		machine = Autonomy_State_Machine()
		machine.run()
	except rospy.ROSInterruptException as er:
		rospy.logerr(str(er))
	else:
		pass
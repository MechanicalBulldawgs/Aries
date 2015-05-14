#!/usr/bin/env python

import antigravity, time

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
		
		self.dumpStatePublishedFlag = True
		self.dumpTime = 0
		
		# Load topics
		hopper_state_topic = rospy.get_param("topics/hopper_state", "hopper_state")
		dump_cmds_topic = rospy.get_param("topics/dump_cmds", "dump_cmds")
		
		
		# Setup subscriptions
		rospy.Subscriber(hopper_state_topic, self.hopper_state_callback)
		
		# Setup publishers
		self.dump_pub = rospy.Publisher(dump_topic, String, queue_size = 10)
		
		self.current_state = stateStart

	def run(self):
		'''
		STATE MACHINE!
		'''
		while not rospy.is_shutdown():
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
		pass

	def mine_state(self):
		'''
		This state is responsible for mining dirt.  The robot will mine X number of waypoints in this state.
		'''
		pass

	def nav_bin_state(self):
		'''
		This state is responsible for navigating to the collection bin (backwards).
		Also ensuring the robot is close enough to collection bin for dumping.
		'''
		pass

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
		
		if self.current_state == stateStart:
			pass
		elif self.current_state == stateNavMining:
			pass
		elif self.current_state == stateMine:
			pass
		elif self.current_state == stateNavBin:
			pass
		elif self.current_state == stateDump:
			#when leaving the dump state make self.dumpStatePublishedFlag = False
			if self.hopper_state == "RESTING":
				if ((self.dumpTime + 10) == time.time()):
					self.current_state = stateNavMining
					self.dumpStatePublishedFlag = False
			
	def hopper_state_callback(self, state):
		self.hopper_state = state
		

if __name__ == "__main__":
	try:
		machine = Autonomy_State_Machine()
	except rospy.ROSInterruptException as er:
		rospy.logerr(str(er))
	else:
		pass
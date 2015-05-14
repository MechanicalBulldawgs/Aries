#!/usr/bin/env python

import antigravity

class Autonomy_State_Machine(object):

    def __init__(self):
        '''
        '''
        rospy.init_node("autonomy_node")
        
        self.current_state = None

    def run(self):
        '''
        STATE MACHINE!
        '''
        pass
        # STATE: START

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
        pass

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
        pass

if __name__ == "__main__":
    try:
        machine = Autonomy_State_Machine()
    except rospy.ROSInterruptException as er:
        rospy.logerr(str(er))
    else:
        pass
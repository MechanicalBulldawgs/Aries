import rospy
import time
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import String 
from std_msgs.msg import Int16


class hopp_coll_commands(object):
    """Determines the state of the collector, and the hopper."""

    def __init__(self):

        #Hopper Angle Constansts
        self.HOPPER_MAX = 180
        self.HOPPER_MIN = 75

        self.HOPPER_REST_MAX = 85
        self.HOPPER_DUMP_MIN = 175

        #Collector Angle Constants
        self.COLLECTOR_MAX = 300
        self.COLLECTOR_MIN = 245

        self.COLLECTOR_REST_MAX = 255
        self.COLLECTOR_DUMP_MIN = 295

        #Motor Command constants CHECK THESE VALUES!!!
        self.HOPPER_UP = 385
        self.HOPPER_DOWN = 355
        self.HOPPER_HALT = 370

        #Initializes the node
        rospy.init_node('hopp_coll_command')

        # Initializes the hopper/collector angles to be considered in rest postion. This prevents log errors since the arduino 
        # does not immediatly send the angles. 
        self.hopper_angle = 80
        self.collector_angle = 250
        self.scoop_safe_bool = 1   #CHANGE THIS VALUE

        self.hopper_state = "Resting" #used for keeping track of previous state
        self.collector_state = "Resting"
        self.scoop_safe_state = "Safe"

        self.hopper_command = 370 #initial stall value, used for keeping track of hopper direction
        self.collector_spin_command = 370
        self.collector_tilt_command = 370

        #set up publishers to publish current state of robot
        self.hopper_state_pub = rospy.Publisher("hopper_state", String, queue_size = 10)
        self.collector_state_pub = rospy.Publisher("collector_state", String, queue_size = 10)
        self.scoop_safe_state_pub = rospy.Publisher("scoop_safe_state", String, queue_size = 10)

        #Set up publishers to publish commands to command topic
        self.hopper_cmds_pub = rospy.Publisher("hopper_cmds", Int16, queue_size = 10)
        self.collector_spin_cmds_pub = rospy.Publisher("collector_spin_cmds", Int16, queue_size = 10)
        self.collector_tilt_cmds_pub = rospy.Publisher("collecter_tilt_cmds", Int16, queue_size = 10)

        #Sets up subscribers to get the angles of the collector/hopper
        rospy.Subscriber("hopper_pot", UInt16, self.hopper_callback)
        rospy.Subscriber("collector_pot", UInt16, self.collector_callback)
        rospy.Subscriber("scoop_safe", Bool, self.scoop_safe_callback)

    #Sets the data from the hopper_callback to data member
    def hopper_callback(self, angle):
        self.hopper_angle = angle.data

    #Sets the data from the collector_callback to data member
    def collector_callback(self, angle):
        self.collector_angle = angle.data

    #Sets the data from the IR distance interrupt to data member
    def scoop_safe_callback(self, dist):
        self.scoop_safe_dist = dist.data   
      
    def check_hopper(self, prev_state, prev_command): #SHOULD WE PASS IN PREV ANGLE AND STATE HERE

        command = prev_command
        state = prev_state

        if state == "Resting":
            if self.HOPPER_MIN <= self.hopper_angle <= self.HOPPER_REST_MAX:  #if in resting position
                if prev_command == HOPPER_DOWN:
                    command == HOPPER_HALT
                    #should we somehow signal to drive here?
                else:
                    command = self.HOPPER_UP
                    state = "Resting"
            elif self.HOPPER_REST_MAX < self.hopper_angle <= self.HOPPER_DUMP_MIN: #if in transitioning period
                command = self.HOPPER_UP
                state = "Transitioning"
            else:
                rospy.logerr("Inconsistent hopper angle reading, something went wrong...")
                command = self.HOPPER_HALT
                state = "Warning"
        elif state == "Transitioning":
            if self.HOPPER_MIN <= self.hopper_angle <= self.HOPPER_REST_MAX:  #resting
                command = self.HOPPER_DOWN
                state = "Resting"
            elif self.HOPPER_REST_MAX < self.hopper_angle <= self.HOPPER_DUMP_MIN: #transitioning
                state = "Transitioning"
            elif self.COLLECTOR_DUMP_MIN < self.collector_angle <= self.COLLECTOR_MAX: #dumping
                command = self.HOPPER_UP
                state = "Dumping"
            else:
                rospy.logerr("Inconsistent hopper angle reading, something went wrong...")
                command = self.HOPPER_HALT
                state = "Warning"
        elif state == "Dumping":
            if self.HOPPER_REST_MAX < self.hopper_angle <= self.COLLECTOR_DUMP_MIN:
                command = self.HOPPER_DOWN
                state = "Transitioning"
            elif self.COLLECTOR_DUMP_MIN < self.collector_angle <= self.COLLECTOR_MAX:
                time.sleep(5)      #let dirt fall out. should we do this?
                command = self.HOPPER_DOWN
                state = "Dumping"
            else: 
                rospy.logerr("Inconsistent hopper angle reading, something went wrong...")
                command = self.HOPPER_HALT
                state = "Warning"

        return state, command 
        
    def publish_hopper_state(self, hopper_state):
        state_msg = String()
        state_msg.data = hopper_state
        self.hopper_state_pub.publish(state_msg)

    def publish_hopper_command(self, hopper_command):
        command_msg = Int16()
        command_msg.data = hopper_command
        self.hopper_cmds_pub.publish(command_msg) 

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
        #get commands for hopper and collector
            self.hopper_state, self.hopper_command = self.check_hopper(self.hopper_state, self.hopper_command)
        
            self.publish_hopper_command(self.hopper_command)
            self.publish_hopper_state(self.hopper_state)

            rate.sleep()





if __name__ == "__main__":
    try:
        Hopp_Coll_Command = hopp_coll_commands()
    except rospy.ROSInterruptException as er:
        rospy.logerr(str(er))
    else:
        Hopp_Coll_Command.run()

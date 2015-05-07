

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import String 


class hopp_coll_states(object):
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

        #Initializes the node
        rospy.init_node('hopp_coll_state')

        #Sets up publisher to publish states onto a topics
        self.hopper_state_pub = rospy.Publisher("hopper_state", String, queue_size = 10)
        self.collector_state_pub = rospy.Publisher("collector_state", String, queue_size = 10)

        # Initializes the hopper/collector angles to be considered in rest postion. This prevents log errors since the arduino 
        # does not immediatly send the angles. 
        self.hopper_angle = 80
        self.collector_angle = 250

        #Sets up subsribers to get the angles of the collector/hopper
        rospy.Subscriber("pot_hopper", UInt16, self.hopper_callback)
        rospy.Subscriber("pot_collector", UInt16, self.collector_callback)

        

    #Sets the data from the hopper_callback to data member
    def hopper_callback(self, data):
        self.hopper_angle = data.data

    #Sets the data from the hopper_callback to data member
    def collector_callback(self, data):
        self.collector_angle = data.data

    #Checks the hopper's current angle and determines what state it is in.
    def check_hopper(self):
        if self.HOPPER_MIN <= self.hopper_angle <= self.HOPPER_REST_MAX: 
            state = "Resting"
        elif self.HOPPER_REST_MAX < self.hopper_angle <= self.HOPPER_DUMP_MIN:
            state = "Transitioning"
        elif self.HOPPER_DUMP_MIN < self.hopper_angle <= self.HOPPER_MAX:
            state = "Dumping"
        else:
            state = "Warning"
            #print self.hopper_angle
            rospy.logerr("Hopper Angle: Out of Bounds. Hopper may be in dangerous position.")

        return state

    #Publishes the hopper state to it's respective topic
    def publish_hopper_state(self, hopper_state):
        state_msg = String()
        state_msg.data = hopper_state
        self.hopper_state_pub.publish(state_msg)

    #Checks the collector's current angle and determines what state it is in.
    def check_collector(self):
        if self.COLLECTOR_MIN <= self.collector_angle <=self.COLLECTOR_REST_MAX: 
            state = "Resting"
        elif self.COLLECTOR_REST_MAX < self.collector_angle <= self.COLLECTOR_DUMP_MIN:
            state = "Transitioning"
        elif self.COLLECTOR_DUMP_MIN < self.collector_angle <= self.COLLECTOR_MAX:
            state =  "Dumping"
        else:
            print self.collector_angle
            state = "Warning"
            rospy.logerr("Collector Angle: Out of Bounds. Collector may be in dangerous position.")

        return state

    #Publishes the collector state to it's respective topic    
    def publish_collector_state(self, collector_state):
        state_msg = String()
        state_msg.data = collector_state
        self.collector_state_pub.publish(state_msg)


    #Main Fnction. 
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #hopper_state = self.check_hopper()
            collector_state = self.check_collector()

            #self.publish_hopper_state(hopper_state)
            self.publish_collector_state(collector_state)

            rate.sleep()

if __name__ == "__main__":
    try:
        Hopp_Coll_State = hopp_coll_states()
    except rospy.ROSInterruptException as er:
        rospy.logerr(str(er))
    else:
        Hopp_Coll_State.run()



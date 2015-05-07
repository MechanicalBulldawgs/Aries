#!/usr/bin/env python
import rospy
import aries.srv
from nav_msgs.msg import OccupancyGrid

if __name__ == "__main__":
    rospy.init_node("TESTER")
    r = rospy.Rate(10)

    print("Waiting for /aries/get_occupancy_map...")
    rospy.wait_for_service("/aries/get_occupancy_map")
    print("Done waiting for service...")
    pub = rospy.Publisher('/aries/occupancy_map', OccupancyGrid, queue_size=10)
    while not rospy.is_shutdown():
        get_occupancy_map = rospy.ServiceProxy("/aries/get_occupancy_map", aries.srv.occupancy_map)
        pub.publish(get_occupancy_map("Garbage").map)
        r.sleep()


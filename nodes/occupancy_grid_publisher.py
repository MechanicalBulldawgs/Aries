#!/usr/bin/env python
import rospy
import aries.srv
from nav_msgs.msg import OccupancyGrid

if __name__ == "__main__":
    rospy.init_node("occupancy_grid_publisher")
    r = rospy.Rate(10)
    OCCUPANCY_MAP = rospy.get_param("topics/occupancy_map")
    SERVICE = rospy.get_param("services/get_occupancy_map")
    print("Waiting for " + SERVICE)
    rospy.wait_for_service(SERVICE)
    print("Done waiting for service...")
    pub = rospy.Publisher(OCCUPANCY_MAP, OccupancyGrid, queue_size=10)
    while not rospy.is_shutdown():
        get_occupancy_map = rospy.ServiceProxy(SERVICE, aries.srv.occupancy_map)
        pub.publish(get_occupancy_map("Garbage").map)
        r.sleep()


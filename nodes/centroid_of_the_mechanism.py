#!/usr/bin/env python
import rospy
import aries.srv
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud
from MeanShiftCluster import MeanShiftCluster
import numpy as np


class centroid_of_the_mechanism(object):
    def __init__(self):
        rospy.init_node("centroid_of_the_mechanism")

        self.mapWidth = rospy.get_param("map_params/width")
        self.mapHeight = rospy.get_param("map_params/height")
        self.resolution = rospy.get_param("map_params/resolution")
        self.cellWidth = int(round(self.mapWidth / self.resolution))
        self.cellHeight = int(round(self.mapHeight / self.resolution))

        self.pc = PointCloud()

        # Creates publisher for filtered point cloud topic
        self._cloud_pub = rospy.Publisher('/aries/obstacle_centroids', PointCloud, queue_size=5)

        

    def handle_localization_pose(self, pose):
        pose = pose.pose

        bandwidth = 10.0

        get_occupancy_map = rospy.ServiceProxy("/aries/get_occupancy_map", aries.srv.occupancy_map)
        occupancyMap = get_occupancy_map("Garbage").map

        # pose.position.x += self.mapWidth/2.0

        xMin = max(int(round((pose.position.x - 2.0) / self.resolution)), 0)
        xMax = min(int(round((pose.position.x + 1.0) / self.resolution)), self.cellWidth)
        yMin = max(int(round((pose.position.y - 2.0) / self.resolution)), 0)
        yMax = min(int(round((pose.position.y + 1.0) / self.resolution)), self.cellHeight)

        obstaclePoints = []
        for x in xrange(xMin, xMax):
            for y in xrange(yMin, yMax):
                weight = occupancyMap.data[x + (y * self.cellWidth)]
                if weight > 0.1:
                    obstaclePoints.append([x,y])

        # if not obstaclePoints:
        #     return

        dataPts = np.asarray(obstaclePoints).T

        try:
            clustCent, data2cluster, cluster2data = MeanShiftCluster(dataPts, bandwidth, nargout=3)
        except:
            raise ValueError(
                "\nxMin: " + str(xMin) +
                "\nxMax: " + str(xMax) +
                "\nyMin: " + str(yMin) +
                "\nyMax: " + str(yMax) +
                "\nobstaclePoints: " + str(obstaclePoints) +
                "\noccupancyMap: " + str(occupancyMap.data)
                )

        # Set up the header.
        self.pc.header.stamp = rospy.Time.now()
        self.pc.header.frame_id = "map"


        points = []
        for centroid, centroidPoints in zip(clustCent, cluster2data):
            p = Point32()
            p.x = centroid[0] * self.resolution
            p.y = centroid[1] * self.resolution
            p.z = 0 #np.size(centroidPoints) * self.resolution
            points.append(p)

        self.pc.points = points

        self._cloud_pub.publish(self.pc)
        

    def run(self):
        r = rospy.Rate(10)

        print("Waiting for /aries/get_occupancy_map...")
        rospy.wait_for_service("/aries/get_occupancy_map")
        print("Done waiting for service...")

        ROBOPOSE_TOPIC = rospy.get_param("topics/localization_pose", "beacon_localization_pose")
        rospy.Subscriber(ROBOPOSE_TOPIC, PoseStamped, self.handle_localization_pose)

        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':

    centroid_centroid = centroid_of_the_mechanism()
    centroid_centroid.run()
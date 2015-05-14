#!/usr/bin/env python
""" Simple occupancy-grid-based mapping without localization. 

Subscribed topics:
/aries/front_pointcloud

Published topics:
/map 
/map_metadata

Author: Nathan Sprague
Version: 2/13/14
"""
import rospy
from nav_msgs.msg import OccupancyGrid as OGmsg
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud

from tf import TransformListener

import numpy as np

from aries.srv import occupancy_map, occupancy_mapRequest, occupancy_mapResponse

class OccupancyGrid(object):
    '''
    '''
    def __init__(self,
                width,
                height,
                resolution,
                pointcloud,
                get_occupancy_map,
                alpha=0.5,
                origin_x=0,
                origin_y=0):
        '''
        Constructor for OccupancyGrid class 
        width: disired width in meters of occupancy grid  (x axis)
        height: desired height in meters of occupancy grid (y axis)
        resolution: desired meter representation of each cell in grid (each cell with be rxr)
        
        Grid is indexed as follows: [y][x] ([height][width])
        [ [ ................. ]
          [ ................. ]
          [ ................. ] ]  

        Each grid cell contains the probability of 
        '''

        # Creates the ROS node.
        rospy.init_node("map_pointcloud")

        self.tf = TransformListener()
        
        
        # Inits the pointcloud Subscriber
        rospy.Subscriber(pointcloud, PointCloud, self.pointcloud_callback)

        # Initialize service that gets the current angle of the lidar
        self._service = rospy.Service(get_occupancy_map, occupancy_map, self.handle_get_occupancy_map)

        self.width = width 
        self.height = height
        self.resolution = resolution
        self.alpha = alpha
        self.origin_x = origin_x
        self.origin_y = origin_y

        self.grid = None
        self._construct_grid(self.width, self.height, self.resolution)


    def _construct_grid(self, width, height, resolution):
        '''
        Private helper function.  Uses width, height, and resolution variables to 
         construct and set self.grid 
        '''
        self.grid = np.zeros((height / resolution, width / resolution))
        # self.grid.fill(-1)

    def _loc_to_indices(self, loc):
        '''
        Transforms global location coordinates (given in meters as x, y position tuple) to grid indices
        '''
        x = int(round(loc[0] / self.resolution))
        y = int(round(loc[1] / self.resolution))
        return (x, y)

    def set_cell(self, loc, value):
        '''
        Given a tuple location (x, y) and a value, set that location in grid to value.
         - Location is given in meters (global x, y location from localization routine)
         - This function transforms given location to grid indices
         - Value should be a probability given as a number between 0 and 1
        '''
        i = self._loc_to_indices(loc)
        self.grid[i[1]][i[0]] = value 

    def get_cell(self, loc):
        '''
        Given global location in meters, return value at that location in occupancy grid
        '''
        i = self._loc_to_indices(loc)
        return self.grid[i[1]][i[0]]

    def handle_get_occupancy_map(self, req):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
     
        grid_msg = OGmsg()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width // self.resolution + 1
        grid_msg.info.height = self.height // self.resolution + 1
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape(self.grid.size) * 100
        grid_msg.data = [int(x) for x in np.round(flat_grid).tolist()]
        print grid_msg.data
        return occupancy_mapResponse(grid_msg)

    def pointcloud_callback(self, cloud):
        # Learning Rate: should be between 0 and 1
        alpha = self.alpha

        # Transforms the point cloud into the /map frame for mapping
        self.tf.waitForTransform("front_laser", "map", rospy.Time(0), rospy.Duration(4.0))
        cloud = self.tf.transformPointCloud("map", cloud)

        for point in cloud.points:
            if (0 < point.x < self.width and 0 < point.y < self.height):
                probability = min(abs(3*point.z), 1.0)*alpha + self.get_cell((point.x, point.y))*(1-alpha)
                self.set_cell((point.x, point.y), probability)


    def run(self):
        # Runs while shut down message is not recieved.
        rate = rospy.Rate(10)
        # Due to differences in startup time, the node needs to wait or it will raise errors
        # by calling for tf transforms at times before startup of the tf broadcaster.
        rospy.sleep(5)
        
        # Waits until a transform is available
        self.tf.waitForTransform("front_laser", "map", rospy.Time(0), rospy.Duration(4.0))
        
        # Main message processing loop
        while not rospy.is_shutdown():
            rate.sleep()



if __name__ == '__main__':
    cellWidth = rospy.get_param("map_params/width")
    cellHeight = rospy.get_param("map_params/height")
    cellResolution = rospy.get_param("map_params/resolution")
    learningRate = rospy.get_param("map_params/learningRate")
    POINTCLOUD = rospy.get_param("topics/filtered_pointcloud")
    GET_OCCUPANCY_MAP = rospy.get_param("topics/get_occupancy_map")
    mpc = OccupancyGrid(width=cellWidth,
                        height=cellHeight,
                        resolution=cellResolution,
                        alpha=learningRate.
                        pointcloud=POINTCLOUD,
                        get_occupancy_map=GET_OCCUPANCY_MAP)
    mpc.run()

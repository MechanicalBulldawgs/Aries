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
from geometry_msgs.msg import Pose, Point, Quaternion, Point32
from sensor_msgs.msg import PointCloud

import numpy as np
from scipy import stats

class OccupancyGrid(object):
    '''
    '''
    def __init__(self, width, height, resolution, origin_x=0, origin_y=0):
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

        self.width = width 
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

        self.grid = None
        self._construct_grid(self.width, self.height, self.resolution)


    def _construct_grid(self, width, height, resolution):
        '''
        Private helper function.  Uses width, height, and resolution variables to 
         construct and set self.grid 
        '''
        self.grid = np.zeros((height / resolution + 1, width / resolution + 1))
        self.grid.fill(-1)

    def _loc_to_indices(self, loc):
        '''
        Transforms global location coordinates (given in meters as x, y position tuple) to grid indices
        '''
        x = int(loc[0] / self.resolution)
        print(x)
        if loc[0] / self.resolution - x >= 0.5: x += 1
        print(x)
        y = int(loc[1] / self.resolution)
        print(y)
        if loc[1] / self.resolution - y >= 0.5: y += 1
        print(y)
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

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
     
        grid_msg = OGmsg()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        grid_msg.data = list(np.round(flat_grid))
        return grid_msg



if __name__ == '__main__':
    pass
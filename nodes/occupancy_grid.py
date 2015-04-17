#!/usr/bin/env python
import numpy

'''
This module implements the occupancy grid for mapping in NASA RMC
'''
####################################
# Constants
MAP_WIDTH = 4.0  # width of occupancy grid in meters
MAP_HEIGHT = 8.0 # height of occupancy grid in meters
####################################

class OccupancyGrid(object):
    '''
    '''
    def __init__(self):
        '''
        '''
        self.grid = None

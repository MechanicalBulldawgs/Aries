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
    def __init__(self, width, height, resolution):
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

        self.grid = None
        self._construct_grid(self.width, self.height, self.resolution)

    def _construct_grid(self, width, height, resolution):
        '''
        Private helper function.  Uses width, height, and resolution variables to 
         construct and set self.grid 
        '''
        self.grid = numpy.zeros((height / resolution + 1, width / resolution + 1))
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




if __name__ == "__main__":
    '''
    some test code for occupancy grid 
    '''
    mappy = OccupancyGrid(5, 5, .5)
    mappy.set_cell(loc = (3.4, 0), value = 0)
    mappy.set_cell((3.9, 4), 0)
    print(str(mappy.grid))
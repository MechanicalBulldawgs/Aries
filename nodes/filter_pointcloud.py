#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid as OGmsg
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion, Point32
from sensor_msgs.msg import PointCloud

from tf import TransformListener

import numpy as np


class Filter_PointCloud(object):
    """ 
    The Filter_PointCloud class filters erroneous cloud data while it's still in
    2D space.
    """
    
    def __init__(self):
        """ Start the mapper. """

        rospy.init_node('filter_pointcloud')

        self.tf = TransformListener()

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering cloud messages.  This is important because the
        # callback is likely to be too slow to keep up with the cloud
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old clouds.  Better to just drop
        # old clouds and always work with the most recent available.
        rospy.Subscriber('/aries/front_pointcloud',
                         PointCloud, self.cloud_callback, queue_size=1)

        self.current_cloud = PointCloud() # current cloud message
        self.received_cloud = False      # True if we've received a new cloud, false if not

        # Creates publisher for filtered point cloud topic
        self._cloud_pub = rospy.Publisher('/aries/filtered_front_pointcloud', PointCloud, queue_size=10)


    def cloud_callback(self, cloud):
        '''
        This function is called everytime a message is transmitted over /aries/front_pointcloud topic
        '''
        # Update current cloud
        self.current_cloud = cloud
        # Set received cloud flag to True
        self.received_cloud = True

    def process_cloud(self, cloud):
        self.received_cloud = False

        # Extracts coordinates into numpy arrays
        y = np.array([v.x for v in cloud.points])
        x = np.array([v.y for v in cloud.points])

        # Set number of standard deviations to allow coordinates to vary within
        nStd = 4

        N = len(x)
        stop = False

        # Containers for discarded points
        #outlierX = np.array([])
        #outlierY = np.array([])
        
        # Iteratively removes outliers and fine tunes the regression line.
        while not stop and N > 0:
            A = np.vstack([x, np.ones(len(x))]).T
            # Performs linear regression
            slope, intercept = np.linalg.lstsq(A, y)[0]

            # Generates best-fit line
            yLinear = slope*x + intercept

            # Calculate errors of the points
            errors = y - yLinear

            # Determines standard deviation of the errors
            sigErr = np.std(errors)

            # Converts errors into multiples of standard deviations
            magError = (np.absolute(errors)/sigErr)
            
            # Finds the largest outlier and its index
            val = np.amax(magError)
            ind = np.argmax(magError)
            
            # Checks if the largest outlier is outside of the specified bounds
            if(val > nStd):
                # Removes the outlier point
                N -= 1
                #outlierX = np.append(outlierX, x[ind])
                #outlierY = np.append(outlierY, y[ind])
                x = np.delete(x, ind)
                y = np.delete(y, ind)
                # print str(val) + " " + str(nStd)
            else:
                # All remaining points lie within boundaries, exit the loop
                stop = True

        # Updates the cloud message with the new coordinates, minus the outliers/noise
        cloud.points = [Point32(x=outY, y=outX, z=0) for outX, outY in zip(x, y)]

        # Transforms the point cloud into the /map frame for mapping
        self.tf.waitForTransform("/front_laser", "/map", rospy.Time(0), rospy.Duration(4.0))
        cloud = self.tf.transformPointCloud("/map", cloud)

        # Only points with potential obstacles need to be mapped.
        # This removes points from the point cloud within safe z-height ranges
        newPoints = []
        for i, point in enumerate(cloud.points):
            if (abs(point.z) < 0.1):
                cloud.points[i].z = 0
        
        # Publishes the new cloud of mapping points
        self._cloud_pub.publish(cloud)

    def run(self):
        '''
        Main work loop.
        '''
        rate = rospy.Rate(10)
        # Due to differences in startup time, the node needs to wait or it will raise errors
        # by calling for tf transforms at times before startup of the tf broadcaster.
        rospy.sleep(5)
        
        # Waits until a transform is available
        self.tf.waitForTransform("/front_laser", "/map", rospy.Time(0), rospy.Duration(4.0))
        
        # Main message processing loop
        while not rospy.is_shutdown():
            if self.received_cloud:
                self.process_cloud(self.current_cloud)
            rate.sleep()



if __name__ == '__main__':
    try:
        f = Filter_PointCloud()
        f.run()
    except rospy.ROSInterruptException:
        print "interrupt"

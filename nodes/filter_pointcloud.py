#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid as OGmsg
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion, Point32
from sensor_msgs.msg import PointCloud

from tf import TransformListener

import numpy as np
from scipy import stats


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

        y = np.array([v.x for v in cloud.points])
        x = np.array([v.y for v in cloud.points])

        # Set the standard deviation
        nStd = 4

        N = len(x)
        stop = False

        outlierX = np.array([])
        outlierY = np.array([])

        while not stop and N > 0:
            # Do linear regression
            slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)

            # print 'r value', r_value
            # print 'p_value', p_value
            # print 'standard deviation', std_err

            # Generate new data
            yLinear = slope*x + intercept

            # Calculate errors
            errors = y - yLinear

            sigErr = np.std(errors)

            # Look for outliers
            magError = (np.absolute(errors)/sigErr)
            val = np.amax(magError)
            ind = np.argmax(magError)
            if(val > nStd):
                N -= 1
                #outlierX = np.append(outlierX, x[ind])
                #outlierY = np.append(outlierY, y[ind])
                x = np.delete(x, ind)
                y = np.delete(y, ind)
                # print str(val) + " " + str(nStd)
            else:
                stop = True

        cloud.points = [Point32(x=outY, y=outX, z=0) for outX, outY in zip(x, y)]

        self.tf.waitForTransform("/laser", "/map", rospy.Time(0), rospy.Duration(4.0))
        cloud = self.tf.transformPointCloud("/map", cloud)

        newPoints = []
        for i, point in enumerate(cloud.points):
            if (abs(point.z) >= 0.1):
                newPoints.append(cloud.points[i])
        cloud.points = newPoints
        
        self._cloud_pub.publish(cloud)

    def run(self):
        '''
        Main work loop.
        '''
        rate = rospy.Rate(10)
        rospy.sleep(5)
        self.tf.waitForTransform("/laser", "/map", rospy.Time(0), rospy.Duration(4.0))
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
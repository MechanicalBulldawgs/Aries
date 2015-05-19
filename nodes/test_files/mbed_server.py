#!/usr/bin/python

"""This node interfaces with an Arduino to read sensor data """

import rospy, tf, serial, atexit, json
from mbedrpc import *
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, UInt16


class serial_server(object):
    """This class publishes sensor data received from an attached Arduino.
        Sensor data is published in the run() function"""
    def __init__(self):        
        rospy.init_node('mbed_server')

        self.collector_pub = rospy.Publisher("collector_pot", UInt16, queue_size = 10)
        self.hopper_pub = rospy.Publisher("hopper_pot", UInt16, queue_size = 10)

        """Attempt to get parameters from the ROS server and use them to initialize the list 
        of touch sensors and the connection to the Arduino"""

        baudrate = rospy.get_param("baudrates/mbed", 115200)
        port = rospy.get_param('ports/mbed', '/dev/ttyACM0')
        self.mbed = SerialRPC(port, baudrate)
        print("Connected to mbed on port: " + str(port))

        self.hopper_pot = RPCVariable(self.mbed, "Ahopper")
        self.collector_pot = RPCVariable(self.mbed, "Acollector")

        atexit.register(self._cleanup)

    def run(self):
        """Main loop for publishing sensor data"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            hopper_pot_val = self.hopper_pot.read()
            collector_pot_val = self.collector_pot.read()
            try:
                hopper_pot_val = float(hopper_pot_val)
            except:
                pass
            else:
                self.hopper_pub.publish(UInt16(int(hopper_pot_val)))
            try:
                collector_pot_val = float(collector_pot_val)
            except:
                pass
            else:
                self.collector_pub.publish(UInt16(int(collector_pot_val)))
            rate.sleep()

    
    def _cleanup(self):
        """Called at exit to close connection to Arduino"""
        exit()

if __name__ == "__main__":
    try:
        ser = serial_server()
    except rospy.ROSInterruptException as er:
        rospy.logerr(str(er))
    else:
        ser.run()

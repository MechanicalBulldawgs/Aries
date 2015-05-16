#!/usr/bin/python

"""This node interfaces with an Arduino to read sensor data """

import rospy, tf, serial, atexit, json
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, UInt16


class serial_server(object):
    """This class publishes sensor data received from an attached Arduino.
        Sensor data is published in the run() function"""
    def __init__(self):
        self.accel_data = {"x":0, "y":0, "z":0, "pitch":0, "roll":0}
        self.gyro_data = {"x":0, "y":0, "z":0}
        self.pot_data = {"pot_hopper":0, "pot_collector":0}
        self.ir_data = 0
        
        rospy.init_node('serial_server')
        self.imu_pub = rospy.Publisher("imu", Imu, queue_size = 10)
        self.scoop_safety_pub = rospy.Publisher("scoop_safety", Bool, queue_size = 10)
        self.collector_pub = rospy.Publisher("collector_pot", UInt16, queue_size = 10)
        self.hopper_pub = rospy.Publisher("hopper_pot", UInt16, queue_size = 10)

        """Attempt to get parameters from the ROS server and use them to initialize the list 
        of touch sensors and the connection to the Arduino"""

        port = "/dev/ttyACM3"#rospy.get_param("ports/sensor_arduino", "/dev/ttyUSB0")
        baudrate = rospy.get_param("baudrates/sensor_arduino", 115200)
        print("Attempting to Connect to Arduino on port: " + str(port))

        while not rospy.is_shutdown():
            try:
                self.arduino = serial.Serial(port, baudrate, timeout = 1)
            except:
                print("Failed to connecto to arduino.  Will continue trying.")
                rospy.sleep(3)
            else:
                break
        print("Connected to Arduino on port: " + str(port))
        atexit.register(self._cleanup)


    def run(self):
        """Main loop for publishing sensor data"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            if not self.arduino.inWaiting() > 0:
                rate.sleep()
                continue
            #print("Received data")
            self.parse_data(self.arduino.readline())
            
    def parse_data(self, data):
        try:
            json_str = json.loads(data)["data"]
        except:
            return
        if "imu" in json_str.keys():
            if "accel" in json_str["imu"].keys():
                self.accel_data = json_str["imu"]["accel"]
            if "gyro" in json_str["imu"].keys():
                self.gyro_data = json_str["imu"]["gyro"]
            self.publish_imu()
        if "potentiometers" in json_str.keys():
            self.pot_data = json_str["potentiometers"]
            self.publish_potentiometers()
        if "ir" in json_str.keys():
            self.ir_data = json_str["ir"]
            self.publish_ir()

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'base_link'
        #Orientation from Accelerometer
        roll = self.accel_data["roll"]
        pitch = self.accel_data["pitch"]
        yaw = 0
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        imu_msg.orientation = Quaternion(*q)
        #Angular Velocity from Gyroscope
        imu_msg.angular_velocity.x = self.gyro_data["x"]
        imu_msg.angular_velocity.y = self.gyro_data["y"]
        imu_msg.angular_velocity.z = self.gyro_data["z"]
        #Linear Acceleration from Accelerometer
        imu_msg.linear_acceleration.x = self.accel_data["x"]  
        imu_msg.linear_acceleration.y = self.accel_data["y"]  
        imu_msg.linear_acceleration.z = self.accel_data["z"]   
        self.imu_pub.publish(imu_msg)

    def publish_potentiometers(self):
        collector_msg = UInt16(self.pot_data["pot_collector"])
        hopper_msg = UInt16(self.pot_data["pot_hopper"])
        self.collector_pub.publish(collector_msg)
        self.hopper_pub.publish(hopper_msg)

    def publish_ir(self):
        status_msg = Bool()
        status_msg.data = bool(self.ir_data)
        self.scoop_safety_pub.publish(status_msg)

    def _cleanup(self):
        """Called at exit to close connection to Arduino"""
        self.arduino.close()

if __name__ == "__main__":
    try:
        ser = serial_server()
    except rospy.ROSInterruptException as er:
        rospy.logerr(str(er))
    else:
        ser.run()

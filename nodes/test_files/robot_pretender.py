import socket, cPickle, rospy
from sensor_msgs.msg import Joy


if __name__ == "__main__":
	joy_port = rospy.get_param("control_station_comms/joystick_port")
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.bind(("", joy_port))

	while not rospy.is_shutdown():
		print("waiting...")
		data, addr = sock.recvfrom(4096)
		data = cPickle.loads(data)
		print(data)
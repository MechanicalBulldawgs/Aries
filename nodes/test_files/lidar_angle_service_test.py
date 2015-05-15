import rospy, math
import aries.srv
from std_msgs.msg import Float32

if __name__ == "__main__":
	rospy.init_node("TESTER")
	print("Waiting for get_lidar_pivot_position...")
	rospy.wait_for_service("get_lidar_pivot_position")
	print("Done waiting for service...")
	get_lidar_pivot_position = rospy.ServiceProxy("get_lidar_pivot_position", aries.srv.LidarPivotAngle)
	resp = get_lidar_pivot_position("Garbage")
	print("Angle: " + str(resp.angle))

	pub = rospy.Publisher("lidar_pivot_target_angles", Float32)

	while True:
		resp = get_lidar_pivot_position("Garbage")
		print("Current Angle: " + str(math.degrees(resp.angle)))
		target = raw_input("Enter Angle: ")
		try:
			target = float(target)
		except:
			exit()
		pub.publish(Float32(math.radians(target)))
		rospy.sleep(1)

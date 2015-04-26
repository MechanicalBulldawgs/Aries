import rospy
from geometry_msgs.msg import Point

if __name__ == "__main__":
	rospy.init_node("GOAL_TESTER")
	goal_pub = rospy.Publisher("nav_goal", Point)
	print("Enter 'q' to quit.")
	while not rospy.is_shutdown():
		target = raw_input("Enter point [format: (x,y,z)]: ")
		if target == "q": exit()
		goal_point = Point()
		try:
			target = target.replace("(", "")
			target = target.replace(")", "")
			target = target.split(",")
			x = float(target[0])
			y = float(target[1])
			z = float(target[2])
			goal_point.x = x
			goal_point.y = y
			goal_point.z = z
		except:
			print("poorly formatted input")
		else:
			goal_pub.publish(goal_point)
#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

sensor_msg = None
moving_forward = False
rotating = False

def scan_callback(msg):
	global sensor_msg

	sensor_msg = msg

def send_stop_signal(pub):
	global moving_forward, rotating

	pub.publish(Twist())
	moving_forward = False
	rotating = False

def send_forward_signal(pub):
	global moving_forward

	if moving_forward:
		return

	msg = Twist()
	msg.linear.x = 0.1
	pub.publish(msg)
	moving_forward = True

def send_rotate_signal(pub):
	global rotating

	msg = Twist()
	msg.angular.z = 0.1
	pub.publish(msg)
	rotating = True

def get_front_obstacle_distance(width_degrees):
	global sensor_msg

	width = math.radians(width_degrees)
	idx = int(width / sensor_msg.angle_increment)
	region = []
	for i in range(-idx, idx):
		region.append(sensor_msg.ranges[i])

	print(region)
	return min(region)

def move(pub):
	global sensor_msg, moving_forward, rotating

	front_obstacle_distance = get_front_obstacle_distance(10)

	if front_obstacle_distance < 0.8:
		if moving_forward:
			send_stop_signal(pub)
		elif not rotating:
			send_rotate_signal(pub)
	else:
		if rotating:
			send_stop_signal(pub)
		elif not moving_forward:
			send_forward_signal(pub)


def node():
	global sensor_msg, moving_forward

	rospy.init_node("maze_robot", anonymous=False)

	sub = rospy.Subscriber("/scan", LaserScan, scan_callback)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	rate = rospy.Rate(100)

	while not sensor_msg:
		rospy.sleep(0.5)

	while not rospy.is_shutdown():
		move(pub)
		rate.sleep()


if __name__ == "__main__":
	try:
		node()
	except ROSInterruptException:
		pass


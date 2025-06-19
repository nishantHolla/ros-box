#!/usr/bin/env python
import rospy
import math
from rospy.exceptions import ROSInterruptException

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

current_pose = {'x': 0, 'y': 0, 'theta': 0}
pose_received = False

# Callback to update the current_pose
def update_current_pose(pose):
	global current_pose, pose_received

	# Update the current_pose value and set the pose_received flag to true
	current_pose = { 'x': pose.x, 'y': pose.y, 'theta': pose.theta }
	pose_received = True


# Get the distance between two Pose
def distance_between(start_pose, end_pose):
	a = start_pose['x'] - end_pose['x']
	b = start_pose['y'] - end_pose['y']

	# Euclid's distance formula
	return math.sqrt( (a ** 2) + (b ** 2))


# Get the angle between two Pose
def angle_between(start_pose, end_pose, is_anticlockwise):
	# Normalize the angles and get the difference
	start = start_pose['theta'] % (2 * math.pi)
	end = end_pose['theta'] % (2 * math.pi)
	diff = (end - start) % (2 * math.pi)

	# If is_anticlockwise is set to none then return the shortest angle formed by start and end
	if is_anticlockwise is None:
		angle = min(diff, (2 * math.pi) - diff)

	elif is_anticlockwise:
		angle = diff

	else:
		angle = (2 * math.pi) - diff

	return angle % (2 * math.pi)


# Move the turtle forward by some distance with some speed
def move(distance, speed, publisher):
	global current_pose
	start_pose = current_pose.copy()

	# Define the message and rate of publishing
	message = Twist()
	message.linear.x = speed
	rate = rospy.Rate(20)

	# Keep publishing messages
	while True:
		distance_traveled = distance_between(start_pose, current_pose)
		if distance_traveled >= distance:
			break

		print(f"[MOVE]: \
x={current_pose['x']:.5f} \
y={current_pose['y']:.5f} \
distance to goal={distance - distance_traveled:.5f}"
		)

		publisher.publish(message)
		rate.sleep()

	# Publish reset message to stop the turtle
	publisher.publish(Twist())


# Rotate the turtle by some angle with some speed
def rotate(distance_degrees, speed_degrees, publisher):
	global current_pose
	start_pose = current_pose.copy()

	# Convert speed and distance of rotation to radians and figure out the direction or rotation from
	# speed. Positive speed is anti-clockwise and negative speed is clockwise
	speed = math.radians(speed_degrees)
	distance = math.radians(distance_degrees)
	is_anticlockwise = (abs(speed) / speed) >= 0

	# Define the message and rate of publishing
	message = Twist()
	message.angular.z = speed
	rate = rospy.Rate(100)

	# Keep publishing messages
	while True:
		angle_traveled = angle_between(start_pose, current_pose, is_anticlockwise)
		if angle_traveled >= distance:
			break

		print(f"[ROTATE]: \
x={current_pose['x']:.5f} \
y={current_pose['y']:.5f} \
angle to goal={distance - angle_traveled:.5f}"
		)

		publisher.publish(message)
		rate.sleep()

	# Publish reset message to stop the turtle
	publisher.publish(Twist())


# Go to a specific point in space with some speed
def go_to(x, y, speed, publisher, non_pid_mode=False):
	global current_pose
	goal_pose = { 'x': x, 'y': y }

	# Non PID Mode: First roatate and the move
	if non_pid_mode:
		angle = math.atan2(y - current_pose['y'], x - current_pose['x']) - current_pose['theta']
		direction = 1 if angle > 0 else -1
		angle = abs(angle)
		distance = distance_between(current_pose, goal_pose)

		rotate(math.degrees(angle), 40 * direction, publisher)
		move(distance, speed, publisher)

		return

	# Define the message and rate of publishing
	message = Twist()
	rate = rospy.Rate(100)

	# Constants that determine how fast the turtle moves or rotates
	k_linear = 0.4
	k_angular = 4.0

	# Keep publishing messages
	while True:
		# Calculate the remaining distance and rotation to perform
		distance = distance_between(current_pose, goal_pose)
		angle = math.atan2(y - current_pose['y'], x - current_pose['x'])

		# Stop if distance is less than a threshold
		if distance < 0.01:
			break

		print(f"[GO TO]: \
x={current_pose['x']:.5f} \
y={current_pose['y']:.5f} \
distance to goal={distance:.5f} \
angle to goal={angle:.5f}"
		)

		# Calculate the proportinal speed of movement and rotation
		linear_speed = max(distance, speed)
		angular_speed = angle - current_pose['theta']

		# Update the message parameters
		message.linear.x = linear_speed * k_linear
		message.angular.z = angular_speed * k_angular

		# Publish the message
		publisher.publish(message)
		rate.sleep()

	# Publish reset message to stop the turtle
	publisher.publish(Twist())


# Set the orientation of the turtle
def set_orientation(angle_degrees, speed_degrees, publisher):
	global current_pose
	goal_pose = {'theta': math.radians(angle_degrees)}

	# Get the shortest angle to rotate
	ac_goal = angle_between(current_pose, goal_pose, is_anticlockwise=True)
	c_goal = (2 * math.pi) - ac_goal

	# Call rotate function
	if ac_goal < c_goal:
		rotate(math.degrees(ac_goal), abs(speed_degrees), publisher)
	else:
		rotate(math.degrees(c_goal), -abs(speed_degrees), publisher)

# Move in spiral motion
def spiral(rk, wk, publisher):
	global current_pose

	# Define the message and rate of publishing
	message = Twist()
	rate = rospy.Rate(1)

	# Start publishing messages
	while current_pose['x'] < 10.5 and current_pose['y'] < 10.5:
		print(f"[SPIRAL]: rk={rk:.5f} wk={wk:.5f}")
		rk = rk + 0.1
		message.linear.x = rk
		message.angular.z = wk
		publisher.publish(message)
		rate.sleep()

	# Publish reset message to stop the turtle
	publisher.publish(Twist())

# Define the node action
def node():
	global pose_received

	# Initialize the node
	rospy.init_node('mo_motion', anonymous=False)

	# Setup publisher and subscriber
	pose_sub = rospy.Subscriber('/turtle1/pose', Pose, update_current_pose)
	cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=50)

	# Wait for the first pose
	while not pose_received:
		rospy.sleep(0.5)

	# Move the turtle in a straight line
	move(2, 1, cmd_vel_pub)
	move(2, -1, cmd_vel_pub)

	# Rotate the turtle by an angle
	rotate(90, 50, cmd_vel_pub)
	rotate(90, -50, cmd_vel_pub)
	rotate(270, 80, cmd_vel_pub)
	rotate(270, -80, cmd_vel_pub)

	# Go to specific position
	go_to(1, 1, 2, cmd_vel_pub, non_pid_mode=False)

	# Set orientation
	set_orientation(90, 40, cmd_vel_pub)
	set_orientation(180, 40, cmd_vel_pub)
	set_orientation(200, 40, cmd_vel_pub)
	set_orientation(0, 40, cmd_vel_pub)

	move(8, 3, cmd_vel_pub)
	go_to(5.53, 5.53, 3, cmd_vel_pub);

	# Spiral
	spiral(0, 2, cmd_vel_pub)


# Run the node if the file is executed
if __name__ == '__main__':
	try:
		node()
	except ROSInterruptException:
		pass

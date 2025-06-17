#!/usr/bin/env python

'''
run the turtlesim before running this node
$ rosrun turtlesim turtlesim_node
'''

import rospy
import math
from rospy.exceptions import ROSInterruptException

from geometry_msgs.msg import Twist

# Actions that can be performed by the agent
ACTIONS = [
	"TURN_LEFT",
	"TURN_RIGHT",
	"FORWARD",
]

# Definition of PI for turning left and right
PI = math.pi

# Get the next action to perfrom from the user
def get_next_action():
	selection = -1

	# Keep asking until valid action is entered
	while selection < 0 or selection >= len(ACTIONS):
		print('Type q to quit')
		for i, a in enumerate(ACTIONS):
			print(f'{i}: {a}')

		inp = input('> ')
		if inp == "q":
			return None

		try:
			selection = int(inp)
		except:
			pass

	return ACTIONS[selection]

# Perform the given action by publishing new Twist message using the publisher
def perform_action(action, publiser):
	LINEAR_SPEED = 3        # 3 units per sec
	ANGULAR_SPEED = PI / 2; # 90 degs per sec
	# Create the twist object
	command = Twist()

	# Update the message parameters depending on the action
	if action == "TURN_LEFT":
		command.angular.z = ANGULAR_SPEED # positive is anti clockwise

	elif action == "TURN_RIGHT":
		command.angular.z = -ANGULAR_SPEED # negative is clockwise

	elif action == "FORWARD":
		command.linear.x = LINEAR_SPEED

	# Publish the message
	publiser.publish(command)

	# Do the action of 1 second and then reset
	rospy.sleep(1.0)
	publiser.publish(Twist())

# Define the actions of the node
def node():
	# Initialze the node
	rospy.init_node("turtle_agent", anonymous=False)

	# Create a publisher to /turtle1/cmd_vel
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)

	# Node loop
	while not rospy.is_shutdown():
		action = get_next_action()

		# If no action to perform, stop
		if not action:
			break

		perform_action(action, pub)

# Start the node when the file is executed
if __name__ == "__main__":
	try:
		node()
	except ROSInterruptException:
		pass


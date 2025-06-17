#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException

# Importing String message from std_msgs package
from std_msgs.msg import String

# Callback function to handle new message
def callback(message):
	# To know the structure of message object, run
	# $ rosmsg info std_msgs/String
	rospy.loginfo(message.data)

# Define the actions of the node
def node():
	# Initialize the node
	# - Name of the node
	# - Whether to make the node different from each other
	# - If anonymous is true then a random number is appended to the name of the node for every
	#   instance of the node created using the rosrun command
	rospy.init_node("ps_listener", anonymous=False)

	# Create a subscriber
	# - Name of the topic
	# - Type of the topic
	# - Callback function to execute when a message arrives
	sub = rospy.Subscriber('/my_message', String, callback)

	# Start listening
	rospy.spin()


# Start the node when the file is executed
if __name__ == "__main__":
	try:
		node()
	except ROSInterruptException:
		pass

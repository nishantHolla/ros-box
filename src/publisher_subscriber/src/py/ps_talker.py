#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException

# Importing String message from std_msgs package
from std_msgs.msg import String

# Define the actions of the node
def node():
	# Initialize the node
	# - Name of the node
	# - Whether to make the node different from each other
	# - If anonymous is true then a random number is appended to the name of the node for every
	#   instance of the node created using the rosrun command
	rospy.init_node("ps_talker", anonymous=False)

	# Create a publisher
	# - Name of the topic
	# - Type of the topic
	# - Buffer size to hold messages
	pub = rospy.Publisher('/my_message', String, queue_size = 10)

	# Rate of publication
	rate = rospy.Rate(1) # 1 Hz

	# Start publishing messages
	i = 0
	while not rospy.is_shutdown():
		message = f"hello world {i}"  # Create the message
		rospy.loginfo(message)        # Log the message to stdout
		pub.publish(message)          # Publish the message
		rate.sleep()                  # Sleep for a while so that desired rate of sending is met
		i += 1                        # Increment the counter

# Start the node when the file is executed
if __name__ == "__main__":
	try:
		node()
	except ROSInterruptException:
		pass

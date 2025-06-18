#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException

# Import the service definitions
from services.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse

# Callback for handling the service
def handle_add_two_ints(req):
	rospy.loginfo(req)
	return AddTwoIntsResponse(req.a + req.b)

# Define node actions
def node():
	# Initialize the node
	rospy.init_node("se_server", anonymous=False)

	# Create the service
	# - Name of the service
	# - Type of the service
	# - Callback function to call when request arrives
	service = rospy.Service("add_two_ints", AddTwoInts, handle_add_two_ints)
	print("Started service /add_two_ints")

	# Start listening for requests
	rospy.spin()

# Start the node when the file is executed
if __name__ == '__main__':
	try:
		node()
	except ROSInterruptException:
		pass

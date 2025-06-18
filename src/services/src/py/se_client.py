#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException

# Import the service definitions
from services.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse

# Function to send reqest
def send_request(a, b):
	try:
		# Create a client
		# - Name of the service
		# - Type of the service
		client = rospy.ServiceProxy("add_two_ints", AddTwoInts)

		# Send a request
		response = client(a, b)

		return response.sum

	except rospy.ServiceException(e):
		print(f'Server error: {e}')

# Define node action
def node():
	# Initialize the node
	rospy.init_node("se_client", anonymous=False)

	# Wait for the server to start
	rospy.wait_for_service("add_two_ints")

	# Start sending requests
	while not rospy.is_shutdown():
		try:
			a = int(input("a: "))
			b = int(input("b: "))
		except:
			break

		sum = send_request(a, b)
		print(f'sum: {sum}')


# Start the node if the file is executed
if __name__ == "__main__":
	try:
		node()
	except ROSInterruptException:
		pass

#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException

# Import custom messages
from custom_msgs.msg import IoTSensor

# Display the data to stdout
def display_data(data_point):
	print(f'''
id: {data_point.id}
name: {data_point.name}
temperature: {data_point.temperature}
humidity: {data_point.humidity}
	''')

# Define node action
def node():
	# Initialize the node
	rospy.init_node("cm_screen", anonymous=False)

	# Create a subscriber
	sub = rospy.Subscriber("cm_data", IoTSensor, display_data)

	# Keep listening
	rospy.spin()

# Start the node when the file is executed
if __name__ == "__main__":
	try:
		node()
	except ROSInterruptException:
		pass

#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException
import random

# Import custom messages
from custom_msgs.msg import IoTSensor

# Define node actions
def node():
	# Initialize the node
	rospy.init_node("cm_sensor", anonymous=False)

	# Create a publisher
	pub = rospy.Publisher("cm_data", IoTSensor, queue_size=10)

	# Keep publishing new data
	rate = rospy.Rate(1)
	i = 0
	while not rospy.is_shutdown():
		# Create a random data point
		data = IoTSensor()
		data.id = i
		data.name = "IoT Sensor data"
		data.temperature = random.randint(0, 256)
		data.humidity = random.randint(0, 100)

		# Publish it
		rospy.loginfo(data)
		pub.publish(data)
		rate.sleep()
		i += 1

# Start the node when the file is executed
if __name__ == "__main__":
	try:
		node()
	except ROSInterruptException:
		pass

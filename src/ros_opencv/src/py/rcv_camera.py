#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

running = True

# Bridge to convert images between ros and opencv format
bridge = CvBridge()

# Callback function to process the image frame
def image_callback(ros_image):
	global bridge, running

	# Conver the image from ros to opencv format
	try:
		cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
	except CvBridgeError as e:
		print(e)
		return

	# Process the image here
	cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

	cv2.imshow("Camera", cv_image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		running = False

# Define node actions
def node():
	global running

	# Initialize the node
	rospy.init_node("rcv_camera", anonymous=False)

	# Subscribe to the camera input
	image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

	# Start recording
	while running:
		rospy.sleep(1)

	cv2.destroyAllWindows()

if __name__ == "__main__":
	try:
		node()
	except ROSInterrupException:
		pass

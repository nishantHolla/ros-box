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

def get_contour_center(contour):
	M = cv2.moments(contour)
	cx = -1
	cy = -1

	if M['m00'] != 0:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])

	return cx, cy

def track_ball(frame):
	ball_tracked_frame = frame.copy()

	# Filter colors

	hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	yellow_lower = (30, 150, 100)
	yellow_upper = (50, 255, 255)

	mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)

	# Get contours

	contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	# Draw contours

	for c in contours:
		area = cv2.contourArea(c)
		perimeter = cv2.arcLength(c, True)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		cx, cy = get_contour_center(c)

		if area > 100:
			cv2.circle(ball_tracked_frame, (cx, cy), (int)(radius), (0, 0, 255), 1)

	return ball_tracked_frame

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
	cv_image = track_ball(cv_image)

	cv2.imshow("Ball tracked video", cv_image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		running = False

# Define node actions
def node():
	global running

	# Initialize the node
	rospy.init_node("rcv_ball_video_sub", anonymous=False)

	# Subscribe to the camera input
	image_sub = rospy.Subscriber("/ball_video", Image, image_callback)

	# Start recording
	while running:
		rospy.sleep(1)

	cv2.destroyAllWindows()

# Run the node if the file is executed
if __name__ == "__main__":
	try:
		node()
	except ROSInterrupException:
		pass

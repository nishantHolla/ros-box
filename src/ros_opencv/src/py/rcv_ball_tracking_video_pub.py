#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException
import numpy as np
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# Define node actions
def node():
	global bridge

	video_path = "./videos/ball.mp4"
	video_capture = cv2.VideoCapture(video_path)

	# Initialize the node
	rospy.init_node("rcv_ball_video_pub", anonymous=False)

	# Create publisher
	image_pub = rospy.Publisher("/ball_video", Image, queue_size = 10)

	# Start publishing
	fps = video_capture.get(cv2.CAP_PROP_FPS)
	rate = rospy.Rate(fps if fps > 0 else 30)
	while True:
		ret, frame = video_capture.read()

		if not ret:
			video_capture.release()
			video_capture = cv2.VideoCapture(video_path)
			continue

		image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
		image_pub.publish(image_msg)
		rate.sleep()

# Run the node if the file is executed
if __name__ == '__main__':
	try:
		node()
	except ROSInterruptException:
		pass

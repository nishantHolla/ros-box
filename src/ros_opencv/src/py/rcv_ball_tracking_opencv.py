#!/usr/bin/env python

import numpy as np
import cv2

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

if __name__ == '__main__':
	video_capture = cv2.VideoCapture("./videos/ball.mp4")
	while True:
		ret, frame = video_capture.read()

		if not ret:
			break

		cv2.imshow("Video", frame)
		ball_tracked_frame = track_ball(frame)
		cv2.imshow("Ball tracked video", ball_tracked_frame)

		if cv2.waitKey(10) & 0xFF == ord('q'):
			break

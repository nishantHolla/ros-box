#!/usr/bin/env python

import numpy as np
import cv2

break_line = "-------\n"

print("Setup video capture")
video_capture = cv2.VideoCapture(0)
# Or read from a file
# video_capture = cv2.VideoCapture("./videos/movie.mp4")
print(break_line)

print("Loop through each frame")
while True:
	ret, frame = video_capture.read()

	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	cv2.imshow("Video", frame)
	if cv2.waitKey(10) & 0xFF == ord('q'):
		break

video_capture.release()
cv2.destroyAllWindows()

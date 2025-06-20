#!/usr/bin/env python

import numpy as np
import cv2

break_line = "-------\n"

print("Create image canvas")
img = np.zeros((512, 512, 3), np.uint8)
print(break_line)

print("Draw line")
cv2.line(img,
		 (0,0),       # Start
		 (511, 511),  # End
		 (255, 0, 0), # Color
		 5            # Thickness
)
print(break_line)

print("Draw rectangle")
cv2.rectangle(img,
			  (384, 0),    # Top left
			  (510, 128),  # Bottom right
			  (0, 255, 0), # Color
			  3            # Thickness
)
print(break_line)

print("Draw circle")
cv2.circle(img,
		   (447, 63),   # Center
		   63,          # Radius
		   (0, 0, 255), # Color
		   3            # Thickness
)
print(break_line)

print("Draw ellipse")
cv2.ellipse(img,
			(256,256),   # Center
			(100,50),    # Major axis
			0,
			180,         # Start angle
			360,         # End angle
			(0, 255, 0), # Color
			-1           # Thickness (-1 is filled)
)
print(break_line)

print("Draw polygon")
pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
pts = pts.reshape((-1,1,2))
cv2.polylines(img,[pts],True,(0,255,255))
print(break_line)

print("Draw text")
font = cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(img,'OpenCV',(10,500), font, 4,(255,255,255),2,cv2.LINE_AA)
print(break_line)

cv2.imshow("Image", img)
cv2.waitKey(0)

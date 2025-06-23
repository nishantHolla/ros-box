#!/usr/bin/env python
import numpy as np
import cv2

break_line = "-----------\n"

def get_contour_center(contour):
	M = cv2.moments(contour)
	cx = -1
	cy = -1

	if M['m00'] != 0:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])

	return cx, cy

print("Reading image")
rgb_image = cv2.imread("./images/ball.jpg")
cv2.imshow("Original", rgb_image)
print(break_line)

print("Filtering colors")
hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
cv2.imshow("HSV", hsv_image)

yellow_lower = (30, 150, 100)
yellow_upper = (50, 255, 255)

mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
cv2.imshow("Mask", mask)
print(break_line)

print("Getting contours")
contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(break_line)

print("Drawing contours")
black_image = np.zeros([mask.shape[0], mask.shape[1], 3], 'uint8')

for c in contours:
	area = cv2.contourArea(c)
	perimeter = cv2.arcLength(c, True)
	((x, y), radius) = cv2.minEnclosingCircle(c)

	if area > 100:
		cv2.drawContours(rgb_image, [c], -1, (150, 250, 150), 1)
		cv2.drawContours(black_image, [c], -1, (150, 250, 150), 1)
		cx, cy = get_contour_center(c)
		cv2.circle(rgb_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
		cv2.circle(black_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
		print(f"Area: {area}, Perimeter: {perimeter}")
print(f"Number of contours: {len(contours)}")
cv2.imshow("Contours", rgb_image)
cv2.imshow("Black image contours", black_image)
print(break_line)

cv2.waitKey(0)

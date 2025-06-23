#!/usr/bin/env python

import numpy as np
import cv2

image = cv2.imread("./images/shapes.jpg")
cv2.imshow("Original", image)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imshow("Grayscale", gray)

binary = cv2.adaptiveThreshold(
	gray,
	255,
	cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
	cv2.THRESH_BINARY_INV,
	5,
	2,
)
cv2.imshow("Threshold", binary)

contours, hierarchy = cv2.findContours(binary,
                                          cv2.RETR_TREE,
                                          cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(
    image,       # Image to draw on
    contours,    # Contours to draw
    -1,          # Index of contours to draw, -1 means all
    (255, 0, 0), # Color to draw in
    1,           # Thickness of the line
)
cv2.imshow("Contours", image)

counter = 0
for c in contours:
	area = cv2.contourArea(c)
	perimeter = cv2.arcLength(c, True)
	if area < 1000 or perimeter < 750:
		continue
	((x, y), radius) = cv2.minEnclosingCircle(c)
	print(f'{counter}: Area: {area}, Perimeter: {perimeter} ')
	counter += 1

cv2.waitKey(0)

#!/usr/bin/env python

import numpy as np
import cv2

image = cv2.imread("./images/shapes.jpg")
cv2.imshow("Original", image)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
cv2.imshow("HSV", hsv)

'''
red: 0 to 30
yellow: 30 to 60
green: 60 to 90
cyan: 90 to 120
blue: 120 to 150
magenta: 150 to 180
'''

red = (
    ( 0, 0, 0 ), # Lower bound H, S, V
    ( 30, 255, 255) # Upper bound H, S, V
)

# Define a mask
mask = cv2.inRange(image, red[0], red[1])

cv2.imshow("Color filter", mask)
cv2.waitKey(0)

cv2.waitKey(0)

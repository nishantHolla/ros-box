#!/usr/bin/env python

import numpy as np
import cv2

break_line = "---------\n"

print("Read image file")
img = cv2.imread("./images/tree.jpg", cv2.IMREAD_COLOR)
print(break_line)


print("Display original image")
cv2.imshow("Original Image", img)
cv2.moveWindow("Original Image", 0, 0)
print(break_line)

print("Split the image into three channels")
blue, green, red = cv2.split(img)
print(break_line)

print("Show channel images")
cv2.imshow("Blue channel", blue)
cv2.imshow("Green channel", green)
cv2.imshow("Red channel", red)
print(img.shape)
print(break_line)

cv2.waitKey(0)
cv2.destroyAllWindows()

print("Converting to HSV encoding")
hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
hue, saturation, value = cv2.split(hsv_img)
cv2.imshow("Hue", hue)
cv2.imshow("Saturation", saturation)
cv2.imshow("Value", value)
print(hsv_img.shape)
print(break_line)

cv2.waitKey(0)
cv2.destroyAllWindows()

print("Converting to grayscale")
grayscale_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("Grayscale", grayscale_img)
print(grayscale_img.shape)
print(break_line)

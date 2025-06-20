#!/usr/bin/env python

import numpy as np
import cv2

break_line = "---------\n"

print("Read image")
img = cv2.imread("./images/tree.jpg")
print(break_line)

print("Create a window")
cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
print(break_line)

print("Display the image")
cv2.imshow("Image", img)
print(break_line)

print("Wait for a key to be pressed")
cv2.waitKey(0)
print(break_line)

print("Copy the image")
cv2.imwrite("./images/tree-copy.jpg", img)
print(break_line)


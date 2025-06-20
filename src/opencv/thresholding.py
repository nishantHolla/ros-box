#!/usr/bin/env python
import numpy as np
import cv2

image = cv2.imread("./images/tree.jpg", cv2.IMREAD_GRAYSCALE)
break_line = "---------\n"

print("Global thresholding with binary")
ret, global_binary = cv2.threshold(
	image,            # Image to threshold
	115,              # Threshold value
	255,              # Max value
	cv2.THRESH_BINARY # Method
)
cv2.imshow("Global Binary", global_binary)
print(break_line)

print("Global thresholding with binary inverse")
ret, global_binary_inv = cv2.threshold(
	image,                 # Image to threshold
	115,                   # Threshold value
	255,                   # Max value
	cv2.THRESH_BINARY_INV  # Method
)
cv2.imshow("Global Binary Inverse", global_binary_inv)
print(break_line)

print("Global thresholding with truncation")
ret, global_truncation = cv2.threshold(
	image,           # Image to threshold
	115,             # Threshold value
	255,             # Max value
	cv2.THRESH_TRUNC # Method
)
cv2.imshow("Global Truncation", global_truncation)
print(break_line)

print("Global thresholding with tozero")
ret, global_tozero = cv2.threshold(
	image,            # Image to threshold
	115,              # Threshold value
	255,              # Max value
	cv2.THRESH_TOZERO # Method
)
cv2.imshow("Global tozero", global_tozero)
print(break_line)

print("Adaptive thresholding with mean")
adaptive_mean = cv2.adaptiveThreshold(
	image,                      # Image to threshold
	255,                        # Max value
	cv2.ADAPTIVE_THRESH_MEAN_C, # Method
    cv2.THRESH_BINARY,          # Threshold method
    11,                         # Block size
    2
)
cv2.imshow("Adaptive Mean", adaptive_mean)
print(break_line)

print("Adaptive thresholding with Gaussian")
adaptive_gaussian = cv2.adaptiveThreshold(
	image,                          # Image to threshold
	255,                            # Max value
	cv2.ADAPTIVE_THRESH_GAUSSIAN_C, # Method
    cv2.THRESH_BINARY,              # Threshold method
    11,                             # Block size
    2
)
cv2.imshow("Adaptive Gaussian", adaptive_gaussian)
print(break_line)

cv2.waitKey(0)



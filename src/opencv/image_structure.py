#!/usr/bin/env python

import numpy as np
import cv2

break_line = "----------\n"

print("Read the image")
img = cv2.imread("./images/tree.jpg")
print("----------")

print("Print the image data as multi-dimensional array")
print(img)
print("----------")

print("Print size of the image")
print(img.size)
print("----------")

print("Print shape of the image")
print(img.shape)
height, width, channel = img.shape
print("----------")

print("Print data type of the image values")
print(img.dtype)
print("----------")

print("Print value of specific pixel")
print(img[10][5])
print("----------")

print("Print entire row")
print(img[10])
print("----------")

print("Get individual channel of the image")
red_channel = img[:][:][0]
print(red_channel)
print("----------")

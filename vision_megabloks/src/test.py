#!/usr/bin/env python
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
ret, img = cap.read()

while True:
    for pixel in np.ndenumerate(img):
        if pixel[0] <= pixel[1] and pixel[0] <= pixel[2]
            pixel = [0, 0, 0]
    cv2.imshow(img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()

"""
rgb_image = cv2.imread("/home/selmawanna/Pictures/Screenshot from 2017-12-12 16-13-37.png")

image_copy = rgb_image.copy()
image_copy[:,:,0] = 0
image_copy[:,:,2] = 0

cv2.imshow('help', image_copy)
cv2.waitKey(0)
"""

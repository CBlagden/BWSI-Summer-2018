#!/usr/bin/env python

import rospy
import numpy as np
import cv2



img = cv2.imread('cruz.png')
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

haar = cv2.CascadeClassifier("./haarcascade_frontal_alt.xml")
faces = haar.detectMultiScale(gray_img, scaleFactor=1.1, minNeighbors=5)

for face in faces: 
    x, y, w, h = face 

cv2.imshow('carlos', cruz.png)
print("Made it here")

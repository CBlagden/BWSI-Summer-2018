'''
08/2/18
TEAM 08 
RAHUL, CHASE, WILLIAM
BWSI

LICENSE: https://github.mit.edu/bwsi-racecar-students-2018/gr8-week3-team08/blob/master/LICENSE.md

'''

import cv2
import numpy as np

def nothing(xp): pass

cv2.namedWindow('edit')
cv2.createTrackbar('H min', 'edit', 39, 179, nothing)
cv2.createTrackbar('H max', 'edit', 78, 179, nothing)
cv2.createTrackbar('S min', 'edit', 177, 255, nothing)
cv2.createTrackbar('S max', 'edit', 255, 255, nothing)
cv2.createTrackbar('V min', 'edit', 76, 255, nothing)
cv2.createTrackbar('V max', 'edit', 155, 255, nothing)

# Crop image!
img = cv2.imread('images/4.jpg', 1)
img = img[:720/2,:,:]
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

while 1:
    hmin, smin, vmin = cv2.getTrackbarPos('H min', 'edit'), cv2.getTrackbarPos( 'S min', 'edit'), cv2.getTrackbarPos( 'V min', 'edit')
    hmax, smax, vmax = cv2.getTrackbarPos('H max', 'edit'), cv2.getTrackbarPos( 'S max', 'edit'), cv2.getTrackbarPos( 'V max', 'edit')
    lower_range = np.array([hmin, smin, vmin])
    upper_range = np.array([hmax, smax, vmax])
    
    blur = cv2.GaussianBlur(img, (25,25), 0)
    blurhsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(blurhsv, lower_range, upper_range)
    img2 = cv2.bitwise_and(blurhsv, blurhsv, mask=mask)
    _, contours,_ = cv2.findContours(mask,  cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    max_contour = None
    max_area = 0
    for c in contours:
        _, _, ew, eh = cv2.boundingRect(c)
        # Eccentricity
        e = ew/eh
        area = cv2.contourArea(c)
        if 0.95 < e < 1.05 and area > 30 and area > max_area:
            max_contour = c
            max_area = area
    x, y, w, h = cv2.boundingRect(max_contour)
    cv2.rectangle(img2, (x, y), (x + w, y + h), (0, 0, 255), 2)
    
    mix = cv2.addWeighted(blur, 0.5, img2, 1, 0)
    cv2.imshow('greenlight', mix)
      
    k = cv2.waitKey(1) & 0xFF
    if k == 27: break
exit()

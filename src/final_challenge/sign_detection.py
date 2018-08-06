import cv2
import numpy as np
import math


def nothing():
    pass


def cartesian_to_polar((x1, y1), (x2, y2)):
    x = x2 - x1
    y = y2 - y1
    rho = np.sqrt(x**2 + y**2)
    phi = math.degrees(np.arctan2(y, x))
    return rho, phi


cam = cv2.VideoCapture(0)

while True:
    #img = cv2.imread('sign4.jpg')
    ret_val, img = cam.read()
    img2 = np.copy(img)

    kernel = np.ones((5, 5), np.uint8)

    img = cv2.erode(img, kernel, iterations=1)
    img = cv2.dilate(img, kernel, iterations=1)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # cv2.inRange(img, np.array([0, 0, 0]), np.array([3, 7, 12]))

    edges = cv2.Canny(gray, 75, 150)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, maxLineGap=1)
    angles = []
    goodLines = []
    bestLines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
        r, theta = cartesian_to_polar((x1, y1), (x2, y2))
        angles.append([r, theta, line])

    pairs = []
    for k1, v1 in enumerate(angles):
        if (v1[1] > 0 or v1[1] < 0) and (-55 < v1[1] < -25 or 25 < v1[1] < 55):
            pairs.append([v1[0], v1[1]])
            goodLines.append(v1[2])

    for line in goodLines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img2, (x1, y1), (x2, y2), (255, 0, 0), 2)
    try:
        for k, i in enumerate(goodLines):
            height, width = img.shape[:2]
            # print i[0][0]
            if i[0][0] > width/2 and pairs[k][1]:
                bestLines.append(1 * pairs[k][0] * (45 - 1/(pairs[k][1])))
            else:
                bestLines.append(-1 * pairs[k][0] * (45 - 1/(pairs[k][1])))
    except:
        pass
    # print width
    # print np.mean(bestLines)
    if np.mean(bestLines) > 0:
        print "perhaps Right"
    else:
        print "perhaps Left"
    # print pairs
    cv2.imshow('hough lines', img)
    cv2.imshow('best lines', img2)

    k = cv2.waitKey(1) & 0xFF

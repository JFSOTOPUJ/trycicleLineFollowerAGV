#!/usr/bin/env python
import cv2
import numpy as np


cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))


def nothing(x):
    pass
# Creating a window for later use
cv2.namedWindow('result')

# Starting with 100's to prevent error while masking
h,s,v = 100,100,100

# Creating track bar
cv2.createTrackbar('h', 'result',0,179,nothing)
cv2.createTrackbar('s', 'result',0,255,nothing)
cv2.createTrackbar('v', 'result',0,255,nothing)

while(1):

    _, frame = cap.read()
    cv2.imshow('frame',frame)

    #converting to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (9,9))

    # get info from track bar and appy to result
    h = cv2.getTrackbarPos('h','result')
    s = cv2.getTrackbarPos('s','result')
    v = cv2.getTrackbarPos('v','result')

    # Normal masking algorithm
    lower_blue = np.array([h,s,v])
    upper_blue = np.array([180,255,255])

    mask = cv2.inRange(hsv,lower_blue, upper_blue)
    yellow_m = cv2.moments(mask)
    try:
            yellow_cx, yellow_cy = yellow_m['m10']/yellow_m['m00'], yellow_m['m01']/yellow_m['m00']
    except ZeroDivisionError:
            yellow_cy, yellow_cx = 480, 640/2
    result = cv2.bitwise_and(frame,frame,mask = mask)
    yellow_res = cv2.circle(result,(int(yellow_cx), int(yellow_cy)), 10,(0,0,255),-1)

    cv2.imshow('result',result)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()

cv2.destroyAllWindows()

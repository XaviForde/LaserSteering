import cv2          #openCV module
import numpy as np
import time
import sys
import matplotlib.pyplot as plt
import FindImageObjects as fio

filename = 'testPlantWithAphids'
imBGR = cv2.imread(filename + '.png')

#cv2.imshow('BGR Colour Scheme', imBGR)
imHSV = cv2.cvtColor(imBGR, cv2.COLOR_BGR2HSV)

lower_blue = np.array([50, 200, 100])
upper_blue = np.array([160, 255, 255])
maskHSV = cv2.inRange(imHSV, lower_blue, upper_blue)
#filtAphidHSV = cv2.bitwise_and(imHSV, imHSV, mask = maskHSV)
_,contours,_ = cv2.findContours(maskHSV, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(imHSV, contours, -1, (0, 0, 255), 1)
#print contours
#cnt = contours[0]

for cnt in contours:

    x,y,w,h = cv2.boundingRect(cnt)
    M = cv2.moments(cnt)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    print cx, cy
    print x,y,w,h
    cv2.rectangle(imHSV,(x,y),(x+w,y+h),(255,0,0),1)

cv2.imshow('img',imHSV)
cv2.waitKey(0)






#maskHSV = cv2.rectangle(maskHSV, (250, 250), (255, 255),[ 160, 240, 50], thickness=1, lineType=8, shift=0)
#cv2.imshow('HSV Colour Scheme', imHSV)
#cv2.imshow('Filtered', maskHSV)
#cv2.waitKey(0)
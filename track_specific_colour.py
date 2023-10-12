# from: https://toptechboy.com/tracking-an-object-based-on-color-in-opencv/

import numpy as np
import cv2
print(cv2.__version__)



width = 640
height = 360
# cam=cv2.VideoCapture(0,cv2.CAP_DSHOW)
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
# cam.set(cv2.CAP_PROP_FPS, 30)
# cam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))

cv2.namedWindow('myTracker')
cv2.moveWindow('myTracker', width, 0)

hueLow = 93
hueHigh = 112
satLow = 170
satHigh = 255
valLow = 0
valHigh = 90


while True:
    ignore,  frame = cam.read()
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lowerBound = np.array([hueLow, satLow, valLow])
    upperBound = np.array([hueHigh, satHigh, valHigh])
    myMask = cv2.inRange(frameHSV, lowerBound, upperBound)
    # myMask=cv2.bitwise_not(myMask)
    myObject = cv2.bitwise_and(frame, frame, mask=myMask)
    myObjectSmall = cv2.resize(myObject, (int(width/2), int(height/2)))
    cv2.imshow('My Object', myObjectSmall)
    cv2.moveWindow('My Object', int(width/2), int(height))
    myMaskSmall = cv2.resize(myMask, (int(width/2), int(height/2)))
    cv2.imshow('My Mask', myMaskSmall)
    cv2.moveWindow('My Mask', 0, height)

    cv2.imshow('my WEBcam', frame)
    cv2.moveWindow('my WEBcam', 0, 0)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cam.release()

import time

import numpy as np
import cv2
print(cv2.__version__)

from person_detection_from_cctv_video.send_servo_val_requests import post_servo_value


width = 640
height = 360
# cam=cv2.VideoCapture(0,cv2.CAP_DSHOW)
# cam = cv2.VideoCapture(0)
# cam = cv2.VideoCapture(1)
cam = cv2.VideoCapture(4)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
# cam.set(cv2.CAP_PROP_FPS, 30)
# cam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))

im_width = 640
im_height = 480
im_half_width = int(640 / 2)

hueLow = 93
hueHigh = 112
satLow = 170  # in living room
satLow = 95  # in my room
satHigh = 255
valLow = 0
valHigh = 90

curr_servo_val = 90
post_servo_value(curr_servo_val)
# TODO soon
# servo_control_frame_rate = 10

TIME_MIN_SINCE_LAST_COMMAND = 0.1
time_since_last_servo_command_sent = time.time()


while True:
    ignore,  frame = cam.read()
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lowerBound = np.array([hueLow, satLow, valLow])
    upperBound = np.array([hueHigh, satHigh, valHigh])
    myMask = cv2.inRange(frameHSV, lowerBound, upperBound)
    # myMask=cv2.bitwise_not(myMask)
    myObject = cv2.bitwise_and(frame, frame, mask=myMask)
    myObjectSmall = cv2.resize(myObject, (int(width/2), int(height/2)))
    how_much_extra_to_move_height_of_window = 250
    cv2.imshow('My Object', myObjectSmall)
    cv2.moveWindow('My Object', int(width/2), int(height) + how_much_extra_to_move_height_of_window)
    myMaskSmall = cv2.resize(myMask, (int(width/2), int(height/2)))

    cv2.imshow('My Mask', myMaskSmall)
    cv2.moveWindow('My Mask', 0, height + how_much_extra_to_move_height_of_window)

    if myMask.sum() > 50:
        # LIMIT IT TODO
        # import pdb;pdb.set_trace()
        # print(myMask.sum())

        non_zero_y, non_zero_x = myMask.nonzero()
        y_mean = int(round(non_zero_y.mean()))
        x_mean = int(round(non_zero_x.mean()))

        # cv2.circle(frame, (y_mean, x_mean), 10, (0, 0, 255, 3))
        cv2.circle(frame, (x_mean, y_mean), 10, (0, 0, 255, 3))


        horizontal_distance_to_center_x_bbox = im_half_width - x_mean
        print('distance: ', horizontal_distance_to_center_x_bbox)

        cv2.line(frame, (x_mean, y_mean), 
                    (im_half_width, y_mean), (0, 255, 0), thickness=3)

        if time.time() - time_since_last_servo_command_sent > TIME_MIN_SINCE_LAST_COMMAND:
            time_since_last_servo_command_sent = time.time()
            
            amount_to_rotate_by = 1  # this can be PID'd
            if horizontal_distance_to_center_x_bbox > 0:
                print('distance positive, object to left')
                curr_servo_val += amount_to_rotate_by
            else:
                print('distance negative, object pospost_servo_value(curr_servo_val)t_servo_value(curr_servo_val)to right')
                curr_servo_val -= amount_to_rotate_by

            if curr_servo_val < 0:
                curr_servo_val = 0
            elif curr_servo_val > 180:
                curr_servo_val = 180


            post_servo_value(curr_servo_val)

    

    cv2.imshow('my WEBcam', frame)
    cv2.moveWindow('my WEBcam', 0, 0)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cam.release()

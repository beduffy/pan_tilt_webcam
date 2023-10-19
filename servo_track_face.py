# taken from https://github.com/Star-Clouds/CenterFace/blob/master/prj-python
import time
import os

import cv2
import scipy.io as sio

from centerface import CenterFace
from person_detection_from_cctv_video.send_servo_val_requests import post_servo_value





def camera():
    # TODO below into function
    width = 640
    im_half_width = int(width / 2)
    height = 360
    # cam=cv2.VideoCapture(0,cv2.CAP_DSHOW)
    # cam = cv2.VideoCapture(0)
    # cam = cv2.VideoCapture(1)
    cam = cv2.VideoCapture(4)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    curr_servo_val = 90
    post_servo_value(curr_servo_val)
    # TODO soon
    # servo_control_frame_rate = 10

    TIME_MIN_SINCE_LAST_COMMAND = 0.1
    time_since_last_servo_command_sent = time.time()

    ret, frame = cam.read()
    h, w = frame.shape[:2]
    centerface = CenterFace()

    while True:
        ret, frame = cam.read()

        dets, lms = centerface(frame, h, w, threshold=0.35)

        chosen_center_pix_pos = None
        for det in dets:
            boxes, score = det[:4], det[4]
            cv2.rectangle(frame, (int(boxes[0]), int(boxes[1])), (int(boxes[2]), int(boxes[3])), (2, 255, 0), 1)
            chosen_center_pix_pos = (int(round((boxes[2] + boxes[0]) / 2)), int(round((boxes[3] + boxes[1]) / 2)))
            cv2.circle(frame, (int(boxes[0]), int(boxes[1])), 10, (0, 255, 255))
            cv2.circle(frame, (int(boxes[2]), int(boxes[3])), 10, (255, 255, 255))
            # import pdb;pdb.set_trace()

        for lm in lms:
            for i in range(0, 5):
                cv2.circle(frame, (int(lm[i * 2]), int(lm[i * 2 + 1])), 2, (0, 0, 255), -1)
        
        if chosen_center_pix_pos:
            cv2.circle(frame, chosen_center_pix_pos, 10, (0, 0, 255))

            horizontal_distance_to_center_x_bbox = im_half_width - chosen_center_pix_pos[0]
            print('distance: ', horizontal_distance_to_center_x_bbox)

            cv2.line(frame, (chosen_center_pix_pos[0], chosen_center_pix_pos[1]), 
                        (im_half_width, chosen_center_pix_pos[1]), (0, 255, 0), thickness=3)

            if time.time() - time_since_last_servo_command_sent > TIME_MIN_SINCE_LAST_COMMAND:
                time_since_last_servo_command_sent = time.time()
                
                amount_to_rotate_by = 1  # this can be PID'd TODO
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

        cv2.imshow('out', frame)

        # Press Q on keyboard to stop recording
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam.release()


if __name__ == '__main__':
    camera()

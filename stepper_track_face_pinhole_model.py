# taken from https://github.com/Star-Clouds/CenterFace/blob/master/prj-python
import time
import os
import serial

import cv2
import scipy.io as sio

from centerface import CenterFace
from person_detection_from_cctv_video.send_servo_val_requests import post_servo_value


def send_angle_to_stepper_serial(ser, angle):
    cmd = '{}\n'.format(angle)

    res = ser.write(cmd.encode())
    ser.flush()
    print('Sent: ', cmd)

    r = ser.readline()
    # print('Got serial line: ', r)


def camera():
    # pixel stuff
    # TODO below camera stuff into function
    width = 640
    height = 360
    img_half_width = int(width / 2)
    img_half_height = int(height / 2)
    # cam = cv2.VideoCapture(0, cv2.CAP_DSHOW) # TODO what is dshow?
    cam = cv2.VideoCapture(4)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # full_horizontal_fov = 78.0
    # https://stackoverflow.com/questions/25088543/estimate-visible-bounds-of-webcam-using-diagonal-fov
    full_horizontal_fov = 70.428
    half_horizontal_fov = full_horizontal_fov / 2

   
    ret, frame = cam.read()
    h, w = frame.shape[:2]
    centerface = CenterFace()

    # motor control
    # ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)
    TIME_MIN_SINCE_LAST_COMMAND = 0.1
    # TIME_MIN_SINCE_LAST_COMMAND = 0.01
    # TIME_MIN_SINCE_LAST_COMMAND = 5
    TIME_MIN_SINCE_LAST_COMMAND = 0.3
    # TIME_MIN_SINCE_LAST_COMMAND = 0.5
    # TODO for steppers im taking 300 milisecond to step there?
    time_since_last_servo_command_sent = time.time()
    curr_pan_servo_val = 0
    # curr_tilt_servo_val = 90
    send_angle_to_stepper_serial(ser, curr_pan_servo_val)
    # post_servo_value(curr_pan_servo_val, 0)
    # post_servo_value(curr_tilt_servo_val, 1)

    while True:
        start_loop_time = time.time()
        ret, frame = cam.read()

        dets, lms = centerface(frame, h, w, threshold=0.35)

        chosen_center_pix_pos = None
        for det in dets:
            boxes, score = det[:4], det[4]
            cv2.rectangle(frame, (int(boxes[0]), int(boxes[1])), (int(boxes[2]), int(boxes[3])), (2, 255, 0), 1)
            chosen_center_pix_pos = (int(round((boxes[2] + boxes[0]) / 2)), int(round((boxes[3] + boxes[1]) / 2)))
            remembered_chosen_center_pix_pos = chosen_center_pix_pos   # TODO do I need to remember the angle?
            cv2.circle(frame, (int(boxes[0]), int(boxes[1])), 10, (0, 255, 255))
            cv2.circle(frame, (int(boxes[2]), int(boxes[3])), 10, (255, 255, 255))

        for lm in lms:
            for i in range(0, 5):
                cv2.circle(frame, (int(lm[i * 2]), int(lm[i * 2 + 1])), 2, (0, 0, 255), -1)

        if chosen_center_pix_pos:
            cv2.circle(frame, chosen_center_pix_pos, 10, (0, 0, 255))

            horizontal_distance_to_center_x_bbox =  chosen_center_pix_pos[0] - img_half_width
            vertical_distance_to_center_y_bbox = img_half_height - chosen_center_pix_pos[1]  # TODO flip or?

            normalised_horizontal_dist = horizontal_distance_to_center_x_bbox / img_half_width
            relative_angle_to_center_from_fov = normalised_horizontal_dist * half_horizontal_fov

            # curr_pan_servo_val = relative_angle_to_center_from_fov
            curr_pan_servo_val = relative_angle_to_center_from_fov * 4
            
            cv2.putText(frame, "Normalised hori dist: {:.3f}".format(normalised_horizontal_dist), (10, 30), 0, 0.7, (255, 0, 0))
            cv2.putText(frame, "relative_angle_to_center: {:.3f}".format(relative_angle_to_center_from_fov), (10, 60), 0, 0.7, (255, 0, 0))

            cv2.line(frame, (chosen_center_pix_pos[0], chosen_center_pix_pos[1]), 
                     (img_half_width, chosen_center_pix_pos[1]), (0, 255, 0), thickness=2)
            cv2.line(frame, (chosen_center_pix_pos[0], chosen_center_pix_pos[1]), 
                     (chosen_center_pix_pos[0], img_half_height), (255, 255, 0), thickness=2)

            if time.time() - time_since_last_servo_command_sent > TIME_MIN_SINCE_LAST_COMMAND:
                time_since_last_servo_command_sent = time.time()

                # TODO modulus
                send_angle_to_stepper_serial(ser, curr_pan_servo_val)

                # print('horizontal dist: {}. error: {:.3f}. error_integral * I const: {:.3f}. amount_to_rotate_by: {:.3f}. Flask angle: {:.3f}'.format(
                #     horizontal_distance_to_center_x_bbox, pid_horizontal.error, 
                #     pid_horizontal.I_const * pid_horizontal.error_integral, amount_to_rotate_by, curr_pan_servo_val))
                
        cv2.imshow('out', frame)

        # Press Q on keyboard to stop recording
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Reporting FPS, varies even without servo control, between 16-21
        # FPS is not the bottleneck yet. Controlling stepper fast enough is? Or serial connection?
        # time_taken = time.time() - start_loop_time
        # print('Time taken: {:.3f}. FPS: {:.3f}'.format(time_taken, 1 / time_taken))

    cam.release()

# TODO swap to grequests? async
# TODO multi threading between servo control and webcam?
# TODO python matplotlib plots at the same time?
# TODO full 360 degrees. TODO encoders and DC motors? or brushless or? or two servos on same axis to cover all 360 degrees? that's worth a video by itself, hmmm
# TODO servo is only moving when i tell it to move 3 degrees?!??! get better servo or do what I said above
# TODO higher FPS everything, also network throttle time, calculate camera FPS of face and compare that to servo speed
# TODO tilt
# TODO mechanical structure without books
# TODO if person leaves camera, then do a sentry look around AND/OR keep going that direction
# TODO can centre face or other face detection run on jetson nano?
# TODO try this: https://github.com/AnbuKumar-maker/AI-on-Jetson-Nano/blob/master/ObjectDetection-Motor%20Tracker
# and this: https://github.com/AnbuKumar-maker/AI-on-Jetson-Nano/blob/master/Motion%20Detection%20Surveillance
# https://github.com/AnbuKumar-maker/AI-on-Jetson-Nano/blob/master/AI%20Face%20Tracking%20Robot
# TODO does this make the webcam faster? https://gist.github.com/gaborvecsei/c7e91d6027597a0b8b05b233198cfe5d
# TODO how to do 60 FPS on webcam? https://forum.opencv.org/t/problem-with-webcam-c922-to-configure-60-fps-720p/9366/5 
# TODO need world model of person's velocity and where they were and stuff. Use realsense or not? Maybe depth camera would be better anyway? yes
#  

if __name__ == '__main__':
    camera()

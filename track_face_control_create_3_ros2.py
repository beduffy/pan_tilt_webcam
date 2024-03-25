# taken from https://github.com/Star-Clouds/CenterFace/blob/master/prj-python
import time
import os

import cv2
import scipy.io as sio

from centerface import CenterFace
from person_detection_from_cctv_video.send_servo_val_requests import post_servo_value


class PID_pixels_to_servo_angles():
    def __init__(self, error_integral_rate, use_changing_PID=False) -> None:        
        # self.P_const = 0.28125
        # self.P_const = 0.15
        self.P_const = 0.03  # very good
        self.P_const = 0.01  # gets to ~2 horizontal distance

        # self.I_const = 0.01
        self.I_const = 0.0001

        self.error_integral = 0.0
        self.error_integral_rate = error_integral_rate

        self.use_changing_PID = use_changing_PID

    def get_amount_to_rotate_by(self, pixel_dist):
        # TODO crazy idea if statement below of big P if horizontal distance over 100
        if self.use_changing_PID:
            if pixel_dist > 50:
                P_const = 0.15
            else:
                P_const = 0.01
        else:
            P_const = 0.01
        
        self.error = pixel_dist  # is this the error? TODO. I always struggled with PID units being different e.g. temperature vs temperature. Speed vs position.
        self.error_integral += self.error * (self.error_integral_rate)  # TODO it might not be TIME_MIN_SINCE_LAST_COMMAND
        # TODO understand I better
        
        # amount_to_rotate_by = horizontal_distance_to_center_x_bbox * P_const  # just P
        amount_to_rotate_by = P_const * self.error
        # amount_to_rotate_by = P_const * error + I_const * error_integral
        # print('error_integral:', error_integral)
        # print('amount_to_rotate_by:', amount_to_rotate_by)

        return amount_to_rotate_by




def camera():
    # TODO below camera stuff into function
    width = 640
    height = 360
    img_half_width = int(width / 2)
    img_half_height = int(height / 2)
    # cam = cv2.VideoCapture(0,cv2.CAP_DSHOW) # TODO what is dshow?
    # cam = cv2.VideoCapture(4)
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    TIME_MIN_SINCE_LAST_COMMAND = 0.1
    # TIME_MIN_SINCE_LAST_COMMAND = 0.05
    # TIME_MIN_SINCE_LAST_COMMAND = 1
    # TIME_MIN_SINCE_LAST_COMMAND = 0.3
    time_since_last_servo_command_sent = time.time()
    curr_pan_servo_val = 90
    curr_tilt_servo_val = 90
    # post_servo_value(curr_pan_servo_val, 0)
    # post_servo_value(curr_tilt_servo_val, 1)
    # TODO soon or not?
    # servo_control_frame_rate = 10
    pid_horizontal = PID_pixels_to_servo_angles(TIME_MIN_SINCE_LAST_COMMAND, use_changing_PID=True)
    # pid_vertical = PID_pixels_to_servo_angles(TIME_MIN_SINCE_LAST_COMMAND)

    remembered_chosen_center_pix_pos = None  # so we keep going towards that direction if human is really fast. hmm flaws 

    ret, frame = cam.read()
    h, w = frame.shape[:2]
    centerface = CenterFace()

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

        # if 5 == 4:
        if chosen_center_pix_pos:
        # if chosen_center_pix_pos or remembered_chosen_center_pix_pos:
            # if chosen_center_pix_pos is None and remembered_chosen_center_pix_pos:
            #     chosen_center_pix_pos = remembered_chosen_center_pix_pos  # TODO sometimes works, sometimes does not
            cv2.circle(frame, chosen_center_pix_pos, 10, (0, 0, 255))

            horizontal_distance_to_center_x_bbox = img_half_width - chosen_center_pix_pos[0]
            vertical_distance_to_center_y_bbox = img_half_height - chosen_center_pix_pos[1]  # TODO flip or?
            # vertical_distance_to_center_y_bbox = chosen_center_pix_pos[1] - img_half_height  # TODO flip or?

            cv2.line(frame, (chosen_center_pix_pos[0], chosen_center_pix_pos[1]), 
                     (img_half_width, chosen_center_pix_pos[1]), (0, 255, 0), thickness=2)
            cv2.line(frame, (chosen_center_pix_pos[0], chosen_center_pix_pos[1]), 
                     (chosen_center_pix_pos[0], img_half_height), (255, 255, 0), thickness=2)

            if time.time() - time_since_last_servo_command_sent > TIME_MIN_SINCE_LAST_COMMAND:
                time_since_last_servo_command_sent = time.time()
                
                amount_to_rotate_by = pid_horizontal.get_amount_to_rotate_by(horizontal_distance_to_center_x_bbox)
                curr_pan_servo_val += amount_to_rotate_by
                # amount_to_rotate_by = pid_vertical.get_amount_to_rotate_by(vertical_distance_to_center_y_bbox)
                # curr_tilt_servo_val += amount_to_rotate_by

                # TODO abstract the below to classes/functions
                if curr_pan_servo_val < 0:
                    curr_pan_servo_val = 0
                elif curr_pan_servo_val > 180:
                    curr_pan_servo_val = 180
                # post_servo_value(curr_pan_servo_val, 0)

                if curr_tilt_servo_val < 0:
                    curr_tilt_servo_val = 0
                elif curr_tilt_servo_val > 180:
                    curr_tilt_servo_val = 180
                # post_servo_value(curr_tilt_servo_val, 1)

                print('horizontal dist: {}. vert dist: {}. error_integral * I const: {:.3f}. amount_to_rotate_by: {:.3f}. Flask angle: {:.3f}'.format(
                    horizontal_distance_to_center_x_bbox, vertical_distance_to_center_y_bbox, 
                    amount_to_rotate_by, curr_tilt_servo_val))
                
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = correction  # Scale down to ensure it's not too high

                print('linear.x:', twist_msg.linear.x, ' angular z:',  correction)
                self.cmd_vel_publisher_.publish(twist_msg)

                # print('horizontal dist: {}. vert dist: {}. error_integral * I const: {:.3f}. amount_to_rotate_by: {:.3f}. Flask angle: {:.3f}'.format(
                #     horizontal_distance_to_center_x_bbox, vertical_distance_to_center_y_bbox, 
                #     pid_horizontal.I_const * pid_horizontal.error_integral, amount_to_rotate_by, curr_pan_servo_val))
                
        cv2.imshow('out', frame)

        # Press Q on keyboard to stop recording
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Reporting FPS, varies even without servo control, between 16-21
        # time_taken = time.time() - start_loop_time
        # print('Time taken: {:.3f}. FPS: {:.3f}'.format(time_taken, 1 / time_taken))

    cam.release()


if __name__ == '__main__':
    camera()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import cv2
from centerface import CenterFace

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.track_face)
        self.cam = cv2.VideoCapture(2)
        self.width = 640
        self.img_half_width = self.width // 2
        self.height = 360
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.centerface = CenterFace()
        self.pid_horizontal = PID_pixels_to_servo_angles(self.timer_period, use_changing_PID=True)

    def track_face(self):
        ret, frame = self.cam.read()
        # print(frame)
        if not ret:
            self.get_logger().warn('No frame captured')
            return
        h, w = frame.shape[:2]
        dets, lms = self.centerface(frame, h, w, threshold=0.35)

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

            horizontal_distance_to_center_x_bbox = self.img_half_width - chosen_center_pix_pos[0]

            cv2.line(frame, (chosen_center_pix_pos[0], chosen_center_pix_pos[1]), 
                        (self.img_half_width, chosen_center_pix_pos[1]), (0, 255, 0), thickness=2)

            for det in dets:
                boxes = det[:4]
                chosen_center_pix_pos = (int(round((boxes[2] + boxes[0]) / 2)), int(round((boxes[3] + boxes[1]) / 2)))
                horizontal_distance_to_center_x_bbox = int(self.width / 2) - chosen_center_pix_pos[0]
                amount_to_rotate_by = self.pid_horizontal.get_amount_to_rotate_by(horizontal_distance_to_center_x_bbox)
                self.send_twist_command(amount_to_rotate_by)
                break  # Only track the first face detected for simplicity

            cv2.imshow('out', frame)

            # Press Q on keyboard to stop recording
            if cv2.waitKey(1) & 0xFF == ord('q'):
                sys.exit()
            

    def send_twist_command(self, amount_to_rotate_by):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = amount_to_rotate_by
        # self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Sending twist command: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

class PID_pixels_to_servo_angles():
    def __init__(self, error_integral_rate, use_changing_PID=False) -> None:
        self.P_const = 0.01
        self.I_const = 0.0001
        self.error_integral = 0.0
        self.error_integral_rate = error_integral_rate
        self.use_changing_PID = use_changing_PID

    def get_amount_to_rotate_by(self, pixel_dist):
        if self.use_changing_PID:
            P_const = 0.15 if pixel_dist > 50 else 0.01
        else:
            P_const = 0.01
        self.error_integral += pixel_dist * self.error_integral_rate
        return P_const * pixel_dist

def main(args=None):
    rclpy.init(args=args)
    face_tracker = FaceTracker()
    rclpy.spin(face_tracker)
    face_tracker.cam.release()
    face_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# source ~/robot_env/bin/activate 
# before run


import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import cv2
import time
from ultralytics import YOLO


class VisionNavNode(Node):

    def __init__(self):

        super().__init__('vision_navigation')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        import torch
        torch.set_num_threads(4)
        torch.backends.quantized.engine = 'qnnpack'

        self.model = YOLO('/home/pi/model/best.pt')

        self.cap = cv2.VideoCapture(0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.STATE = "START_CURVE"

        self.curve_start = time.time()
        self.final_start = None

        self.TARGET_X = 530
        self.FINAL_FORWARD_TIME = 1.6

        self.lost_counter = 0
        self.tag_seen = False

        self.prev_center = None
        self.last_center = None

        self.timer = self.create_timer(0.05, self.loop)


    def loop(self):

        ret, frame = self.cap.read()

        if not ret:
            return

        results = self.model(frame, imgsz=416, conf=0.4)

        tag_detected = False
        center_x = None

        boxes = results[0].boxes


        # -------------------------
        # เลือก TAG BOX
        # -------------------------

        if boxes is not None and len(boxes) > 0:

            smallest_area = 999999999
            best_box = None

            for box in boxes.xyxy:

                x1,y1,x2,y2 = box

                area = float((x2-x1)*(y2-y1))

                if area < 60000 and area < smallest_area:

                    smallest_area = area
                    best_box = box

            if best_box is not None:

                x1,y1,x2,y2 = best_box

                center_x = float((x1+x2)/2)

                tag_detected = True
                self.tag_seen = True
                self.lost_counter = 0

        else:

            self.lost_counter += 1


        # HOLD detection

        if tag_detected:

            self.last_center = center_x

        elif self.last_center is not None:

            center_x = self.last_center


        # smoothing

        if center_x is not None and self.prev_center is not None:

            center_x = 0.7*self.prev_center + 0.3*center_x

        self.prev_center = center_x


        # -------------------------
        # STATE MACHINE
        # -------------------------

        if self.STATE == "START_CURVE":

            self.send_cmd(0.2, -0.5)

            if time.time() - self.curve_start > 2.5:

                self.STATE = "SEARCH_TAG"


        elif self.STATE == "SEARCH_TAG":

            if not tag_detected:

                self.send_cmd(0.0, 0.4)

            else:

                self.STATE = "ALIGN_TAG"


        elif self.STATE == "ALIGN_TAG":

            if center_x is None:

                self.STATE = "SEARCH_TAG"

            else:

                error = center_x - self.TARGET_X

                if abs(error) < 5:

                    self.STATE = "APPROACH_TAG"

                elif error > 0:

                    self.send_cmd(0.0, -0.4)

                else:

                    self.send_cmd(0.0, 0.4)


        elif self.STATE == "APPROACH_TAG":

            if tag_detected:

                error = center_x - self.TARGET_X

                if abs(error) > 20:

                    self.STATE = "ALIGN_TAG"

                else:

                    if abs(error) < 8:

                        self.send_cmd(0.25, 0.0)

                    elif error > 0:

                        self.send_cmd(0.25, -0.2)

                    else:

                        self.send_cmd(0.25, 0.2)

            else:

                if self.tag_seen and self.lost_counter > 5:

                    self.STATE = "FINAL_FORWARD"
                    self.final_start = time.time()


        elif self.STATE == "FINAL_FORWARD":

            self.send_cmd(0.25, 0.0)

            if time.time() - self.final_start > self.FINAL_FORWARD_TIME:

                self.STATE = "STOP"


        elif self.STATE == "STOP":

            self.send_cmd(0.0, 0.0)


        # debug view

        annotated = results[0].plot()

        cv2.line(
            annotated,
            (self.TARGET_X,0),
            (self.TARGET_X,480),
            (255,0,0),
            2
        )

        cv2.imshow("vision nav", annotated)

        cv2.waitKey(1)



def main():

    rclpy.init()

    node = VisionNavNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
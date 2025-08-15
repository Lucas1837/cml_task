#! /home/lucas/catkin_ws/src/cml/venv/bin/python3

import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import Float32, Bool, Int32
from geometry_msgs.msg import Twist
from collections import deque
import math
import pyrealsense2 as rs


class BagDetector:
    def __init__(self):
        rospy.init_node('bag_detection_node')

        # Parameters
        self.pitch_angle_deg = -15
        self.pitch_angle_rad = math.radians(self.pitch_angle_deg)
        self.flag_pointed = False
        self.bag_side = 0
        self.rotate = False

        # Publishers
        self.distance_pub = rospy.Publisher('/bag_distance', Float32, queue_size=10)
        self.angle_pub = rospy.Publisher('/bag_angle', Float32, queue_size=10)
        self.horizontal_pub = rospy.Publisher('/horizontal_dist', Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/pointing_hand', Int32, self.pointing_hand_callback)
        rospy.Subscriber('/aligned', Bool, self.reach_distance_callback)
        rospy.Subscriber('/rotated', Bool, self.rotate_callback)

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)

        # YOLO model
        self.model = YOLO("/home/lucas/catkin_ws/src/cml/take_bag/src/bag_detection/bestpaperbag.pt").cuda()

        self.top_boxes = deque(maxlen=2)

        # Register cleanup
        rospy.on_shutdown(self.cleanup)

    def cleanup(self):
        """Stops RealSense, closes OpenCV windows, and frees resources."""
        rospy.loginfo("Shutting down BagDetector node cleanly...")
        try:
            self.pipeline.stop()
        except Exception as e:
            rospy.logwarn(f"Error stopping RealSense pipeline: {e}")
        cv2.destroyAllWindows()
        rospy.loginfo("Cleanup complete.")

    def correct_pitch(self, x_prime, y_prime):
        h = math.sqrt(x_prime ** 2 + y_prime ** 2)
        beta = math.atan2(y_prime, x_prime)
        theta = self.pitch_angle_rad + beta
        return h * math.cos(theta), h * math.sin(theta)

    def pointing_hand_callback(self, msg):
        if msg and not self.flag_pointed:
            rospy.loginfo(f"waiting for kobuki to rotate")
            rospy.sleep(4)
            self.bag_side = msg.data
            self.flag_pointed = True

    def reach_distance_callback(self, msg):
        if msg.data:
            rospy.loginfo("Target distance reached. Requesting shutdown...")
            rospy.signal_shutdown("Reached target distance.")
    
    def rotate_callback(self, msg):
        self.rotate = msg.data

    def getting_depth(self, x_frame, y_frame):
        if 0 <= x_frame < self.depth_frame.width and 0 <= y_frame < self.depth_frame.height:
            z_m = self.depth_frame.get_distance(x_frame, y_frame)
        else:
            rospy.logwarn(f"Pixel out of bounds: (cx={x_frame}, cy={y_frame})")
            return None

        X_m, Y_m, Z_m = rs.rs2_deproject_pixel_to_point(self.intr, [x_frame, y_frame], z_m)
        z_prime = Z_m * 1000
        y_prime = -Y_m * 1000
        X_m = X_m * 1000
        z_corr, y_corr = self.correct_pitch(z_prime, y_prime)
        return z_corr, y_corr, X_m

    def process_frames(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try: 
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                self.depth_frame = aligned.get_depth_frame()
                self.color_frame = aligned.get_color_frame()
                if not self.depth_frame or not self.color_frame:
                    rate.sleep()
                    continue

                self.intr = self.depth_frame.profile.as_video_stream_profile().intrinsics
                self.color_image = cv2.flip(np.asanyarray(self.color_frame.get_data()), 0)
                height, width = self.color_image.shape[:2]
                middle_x = width / 2

                results = self.model(self.color_image, verbose=False)[0]
                detections = [
                    (conf, box.xyxy[0].tolist())
                    for box, conf in zip(results.boxes, results.boxes.conf)
                    if conf >= 0.7
                ]

                detections.sort(reverse=True, key=lambda x: x[0])
                self.top_boxes.clear()
                self.top_boxes.extend(detections[:2])

                x_of_bag = y_of_bag = 0
                depth_value = None
                angle = None

                if self.top_boxes :
                    top_center = []
                    for _, box in self.top_boxes:
                        x1, y1, x2, y2 = map(int, box)
                        cv2.rectangle(self.color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(self.color_image, 'Bag detected', (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        top_center.append((center_x, center_y))
                    # self.flag_pointed = True
                    self.rotate = True
                    # self.bag_side = 2
                    if self.flag_pointed and self.rotate:
                        if len(top_center) == 2 and self.bag_side != 0:
                            selected_cor = (
                                max(top_center, key=lambda x: x[0]) if self.bag_side == 2
                                else min(top_center, key=lambda x: x[0])
                            )
                            xc, yc = selected_cor
                            cv2.circle(self.color_image, (xc, yc), 10, (255, 0, 0), -1)
                            point_cloud = self.getting_depth(xc, height - yc)
                            if point_cloud:
                                z_corr, y_corr, x_corr = point_cloud
                                depth_value = float(z_corr)
                                h_error = x_corr
                                x_of_bag, y_of_bag = xc, yc

                        elif len(top_center) == 1:
                            xc, yc = top_center[0]
                            point_cloud = self.getting_depth(xc, height - yc)
                            if point_cloud:
                                z_corr, y_corr, x_corr = point_cloud
                                depth_value = float(z_corr)
                                h_error = x_corr
                                x_of_bag, y_of_bag = xc, yc

                    if x_of_bag != 0 and depth_value is not None:
                        offset = x_of_bag - middle_x
                        angle = math.atan2(offset, y_of_bag)

                    if x_of_bag != 0 and depth_value is not None and angle is not None:
                        # rospy.loginfo(f"z = {depth_value}\nx = {h_error}")
                        self.distance_pub.publish(depth_value)
                        self.horizontal_pub.publish(h_error)
                        self.angle_pub.publish(angle)

                cv2.imshow("Camera Feed", self.color_image)
                cv2.waitKey(1)

            except Exception as e:
                if rospy.is_shutdown():
                    break
                rospy.logerr(f"Error frame_process: {e}")
            rate.sleep()


if __name__ == '__main__':
    try:
        detector = BagDetector()
        detector.process_frames()
    except rospy.ROSInterruptException:
        pass


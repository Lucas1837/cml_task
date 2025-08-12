#! /home/lucas/catkin_ws/src/cml/venv/bin/python3
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import Float32MultiArray ,Float32 , Bool
import threading
import math
import pyrealsense2 as rs
import os
import subprocess

class BagDetector:
    def __init__(self):
        rospy.init_node('handle_detection_node')

        # pitch correction
        self.pitch_angle_deg = -25
        self.pitch_angle_rad = math.radians(self.pitch_angle_deg)

        # YOLO and ROS publisher subscriber
        self.confidence_threshold = 0.70
        self.reach_distance_pub = rospy.Publisher('/bag_loc', Float32MultiArray, queue_size=10)
        # self.horizontal_pub = rospy.Publisher('/horizontal_dist',Float32,queue_size=10)
        # rospy.Subscriber('/reach_distance', Bool,self.reach_callback)
        
        self.script_dir = os.path.dirname(os.path.realpath(__file__))
        self.model_path = os.path.join(self.script_dir, "besthandle.pt")
        self.model = YOLO(self.model_path)

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(cfg)

        # align depth to color
        self.align = rs.align(rs.stream.color)

        # threading & state
        self.image_lock = threading.Lock()
        self.latest_image = None
        self.latest_results = []
        self.running = True

        self.flag2 = True
        self.start = None
        self.open_align = True

        # launch threads
        threading.Thread(target=self.display_feed, daemon=True).start()
        threading.Thread(target=self.run, daemon=True).start()

    def correct_pitch(self, x_prime, y_prime):
        h = math.hypot(x_prime, y_prime)
        beta = math.atan2(y_prime, x_prime)
        theta = self.pitch_angle_rad + beta
        return h * math.cos(theta), h * math.sin(theta)

    def run(self):
        while not rospy.is_shutdown() and self.running:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # grab intrinsics for deprojection
            intr = depth_frame.profile.as_video_stream_profile().intrinsics

            # convert to numpy images
            color_image = np.asanyarray(color_frame.get_data())
            color_image = cv2.flip(color_image,0)
            self.latest_image = color_image.copy()

            height = color_image.shape[0]
            # YOLO detection
            results = self.model(color_image, verbose=False)[0]
            detections = []


            for box in results.boxes.data.cpu().numpy():
                x1, y1, x2, y2, conf, cls = box
                if conf < self.confidence_threshold:
                    continue

                cx = int((x1 + x2) / 2)
                # get depth at pixel (in meters)
                cy = int(math.floor(height - y2))  # flip y 
                if 0 <= cx < depth_frame.width and 0 <= cy < depth_frame.height:
                    z_m = depth_frame.get_distance(cx, cy)
                else:
                    rospy.logwarn(f"Pixel out of bounds: (cx={cx}, cy={cy}), skipping.")
                    continue  # skip to the next detection
                if z_m == 0:
                    continue

                # deproject to 3D point (meters), then convert to millimeters
                X_m, Y_m, Z_m = rs.rs2_deproject_pixel_to_point(intr, [cx, y2], z_m)
                x_prime = Z_m * 1000            # forward (mm)
                y_prime = -Y_m * 1000 + 50        # up + offset (mm)
                x_hor = X_m #(meter)

                # correct for camera pitch
                x_corr, y_corr = self.correct_pitch(x_prime, y_prime)

                # publish
                msg = Float32MultiArray(data=[x_corr, y_corr])
                self.reach_distance_pub.publish(msg)
                # self.horizontal_pub.publish(x_hor)
                rospy.loginfo(f"[PUBLISHED] X: {x_corr:.2f}mm, Y: {y_corr:.2f}mm ,Horizontal = {x_hor:.3f}"
                              f"(raw Z={x_prime:.1f}, Y={y_prime:.1f})")

                detections.append(((int(x1), int(y1), int(x2), int(y2)), conf, cls))

            # share latest frame + detections
            with self.image_lock:
                self.latest_results = detections

    def display_feed(self):
        while not rospy.is_shutdown() and self.running:
            with self.image_lock:
                if self.latest_image is not None:
                    img = self.latest_image.copy()
                    for (x1, y1, x2, y2), conf, cls in self.latest_results:
                        cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
                        cv2.putText(img, f"{int(cls)} {conf:.2f}", (x1, y1-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                    cv2.imshow("Camera Feed", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        BagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

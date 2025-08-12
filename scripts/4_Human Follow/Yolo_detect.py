#! /home/lucas/catkin_ws/src/cml/venv/bin/python3

import rospy
import cv2
import numpy as np
import threading
import queue
import math
import pyzed.sl as sl
from std_msgs.msg import Float32, Bool, Int32 , String
from geometry_msgs.msg import Twist
from smooth_approach import TurtleBotApproach
from ultralytics import YOLO
from collections import deque
import os

# Import MediaPipe for skeleton detection
import mediapipe as mp

class PersonDetector:
    def __init__(self):
        # ROS publishers/subscribers
        self.distance_pub = rospy.Publisher('/person_distance', Float32, queue_size=10)
        self.angle_pub = rospy.Publisher('/person_angle', Float32, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.audio_pub = rospy.Publisher('/text_to_speech',String,queue_size=10)
        rospy.Subscriber('/pointing_hand', Int32, self.pointing_hand_callback)
        rospy.Subscriber('/reach_distance', Bool, self.reach_distance_callback)

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 15
        self.runtime_parameters = sl.RuntimeParameters()
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            rospy.logerr("Failed to open ZED camera.")
            exit(1)

        # Frame queue
        self.frame_queue = queue.Queue(maxsize=10)

        # MediaPipe pose setup
        self.mp_pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6)
        self.mp_drawing = mp.solutions.drawing_utils

        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(script_dir, "yolo12n.pt")
        self.model = YOLO(model_path).cuda()
        self.top_boxes = deque(maxlen=1)
        # Start threads
        self.capture_thread = threading.Thread(target=self.capture)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        self.process_thread = threading.Thread(target=self.process_frames)
        self.process_thread.daemon = True
        self.process_thread.start()


        rospy.on_shutdown(self.cleanup)

    def cleanup(self):
        rospy.loginfo("Shutting down PersonDetector... closing resources.")
        
        try:
            self.zed.close()
            rospy.loginfo("ZED camera closed.")
        except Exception as e:
            rospy.logwarn(f"Error closing ZED camera: {e}")
        try:
            self.mp_pose.close()
        except Exception as e:
            rospy.logwarn(f"Error closing MediaPipe pose: {e}")
        cv2.destroyAllWindows()
        rospy.loginfo("Cleanup complete.")

    def pointing_hand_callback(self, msg):
        self.person_side = msg.data if msg.data else 0

    def reach_distance_callback(self, msg):
        if msg.data:
            self.velocity_pub.publish(Twist())

    def capture(self):
        rate = rospy.Rate(30)
        img_mat = sl.Mat(); depth_mat = sl.Mat()
        while not rospy.is_shutdown():
            if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(img_mat, sl.VIEW.LEFT)
                self.zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
                color = cv2.cvtColor(img_mat.get_data(), cv2.COLOR_BGRA2BGR)
                depth_np = depth_mat.get_data()
                if not self.frame_queue.full():
                    self.frame_queue.put((color, depth_np))
            rate.sleep()

    def process_frames(self):
        self.audio_pub.publish("I will now start to follow you")
        
        while not rospy.is_shutdown():
            if self.frame_queue.empty():
                continue
            image, depth = self.frame_queue.get()

            # Create a mask for depth <= 1800 mm
            depth_mask = (depth <= 1800) & (depth > 0) & (~np.isnan(depth))

            # Convert mask to uint8 for OpenCV
            mask_uint8 = depth_mask.astype(np.uint8) * 255

            # Apply mask to RGB image (keep only close objects)
            filtered_image = cv2.bitwise_and(image, image, mask=mask_uint8)

            mp_depth = None
            yl_depth = None
            y_sh = None
            x_sh = None
            d = None
            # # --- rotate view 180° so it appears upright ---
            # image = cv2.rotate(image, cv2.ROTATE_180)
            # depth  = np.rot90(depth, 2)  # rotate depth map as well (180°)

            h, w = image.shape[:2]
            # Run MediaPipe pose
            self.mp_xy = []
            mp_results = self.mp_pose.process(cv2.cvtColor(filtered_image, cv2.COLOR_BGR2RGB))
            if mp_results.pose_landmarks:
                self.mp_drawing.draw_landmarks(
                    image, mp_results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)

                # Compute spine via mid‑shoulder & mid‑hip
                lm = mp_results.pose_landmarks.landmark
                lh, rh = lm[mp.solutions.pose.PoseLandmark.LEFT_HIP], lm[mp.solutions.pose.PoseLandmark.RIGHT_HIP]
                ls, rs = lm[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER], lm[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]
                conf_hip = (lh.visibility + rh.visibility) / 2.0
                conf_sh  = (ls.visibility + rs.visibility) / 2.0

                if conf_hip > 0.7 or conf_sh > 0.7:
                    x_hip = int((lh.x + rh.x) * 0.5 * w)
                    y_hip = int((lh.y + rh.y) * 0.5 * h)
                    x_sh  = int((ls.x + rs.x) * 0.5 * w)
                    y_sh  = int((ls.y + rs.y) * 0.5 * h)
                    cv2.circle(filtered_image, (x_hip, y_hip), 5, (255, 0, 0), -1)
                    cv2.circle(filtered_image, (x_sh,  y_sh), 5, (0, 255, 0), -1)
                    cv2.line(filtered_image, (x_sh, y_sh), (x_hip, y_hip), (0, 255, 255), 3)

                    # Centerline for reference
                    cv2.line(filtered_image, (w//2, 0), (w//2, h), (200,200,200), 1)
                    self.mp_xy = [x_sh,y_sh]
                    # if 0 <= y_hip < depth.shape[0] and 0 <= x_hip < depth.shape[1]:
                    #     d = float(depth[y_hip, x_hip])
                    #     if not math.isnan(d) and d > 0:
                    #         offset = x_hip - w//2
                    #         angle  = math.atan2(offset, y_hip)
            if y_sh != None and 0 <= y_sh < depth.shape[0] and 0 <= x_sh < depth.shape[1]:
                mp_depth = float(depth[y_sh, x_sh])
            if mp_depth == None or mp_depth<700 :
                #Yolo result
                yl_results = self.model(cv2.cvtColor(filtered_image, cv2.COLOR_BGR2RGB), stream=False,verbose = False)
                detections = []
                self.yl_xy = []
                for boxes in yl_results[0].boxes:  
                        confidence = boxes.conf[0]
                        class_id = int(boxes.cls[0])
                        if confidence > 0.80 and class_id ==0:
                            box = boxes.xyxy[0].tolist()  # [x1, y1, x2, y2]
                            detections.append((confidence, box))

                # Sort and store top 1 confidence boxe
                detections.sort(reverse=True, key=lambda x: x[1])  # Sort by confidence
                self.top_boxes.clear()
                self.top_boxes.extend(detections[:1])  # Keep only top 1
                
                # # MediaPipe: Set x_sh/y_sh and depth if valid
                # if hasattr(self, 'mp_xy') and len(self.mp_xy) == 2:
                #     x_sh, y_sh = self.mp_xy
                #     if (0 <= y_sh < depth.shape[0] and 0 <= x_sh < depth.shape[1]):
                #         d_val = depth[y_sh, x_sh]
                #         if not math.isnan(d_val) and d_val > 0:
                #             mp_depth = float(d_val)

                if self.top_boxes:
                    x1, y1, x2, y2 = map(int, self.top_boxes[0][1])
                    cv2.rectangle(filtered_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(filtered_image, 'Yolo detected', (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    yl_x, yl_y = (x1 + x2) // 2, (y1 + y2) // 2
                    if (0 <= yl_y < depth.shape[0] and 0 <= yl_x < depth.shape[1]):
                        d_val = depth[yl_y, yl_x]
                        if not math.isnan(d_val) and d_val > 0:
                            self.yl_xy = [yl_x,yl_y]
                            yl_depth = float(d_val)

                if len(self.yl_xy) != 0:
                    yl_depth = float(depth[self.yl_xy[1],self.yl_xy[0]]) 
                    rospy.loginfo("using YOLO")
                    d = yl_depth
                    x_d, y_d = yl_x, yl_y
            

            else:  # Use MediaPipe by default
                    rospy.loginfo("using Mediapipe")
                    d = mp_depth
                    x_d, y_d = x_sh, y_sh

            if d is not None and d > 0:
                offset = x_d - w//2
                angle  = math.atan2(offset, y_d)
                rospy.loginfo(f"distance = {d}")
                rospy.loginfo(f"angle = {angle}")
                self.distance_pub.publish(d)
                self.angle_pub.publish(angle)
                # overlay text + print
                # txt = f"{d/1000:.2f} m, {angle:.2f} rad"
                # cv2.putText(image, txt, (x_hip+10, y_hip-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                # print("Spine:", txt)
            cv2.imshow('Human_detect', filtered_image)
            cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('skeleton_pose_node')
    detector = PersonDetector()
    turtlebot = TurtleBotApproach()
    turtlebot.run()

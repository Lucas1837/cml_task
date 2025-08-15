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
        init_params.camera_fps = 30
        self.runtime_parameters = sl.RuntimeParameters()
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            rospy.logerr("Failed to open ZED camera.")
            exit(1)

        # Frame queue
        self.frame_queue = queue.Queue(maxsize=10)

        self.model = YOLO("/home/lucas/catkin_ws/src/cml/follow_me/scripts/yolo12n.pt")
        self.top_boxes = []
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
        target_id = None  # To lock onto one person
    
        while not rospy.is_shutdown():
            if self.frame_queue.empty():
                continue
            image, depth = self.frame_queue.get()

            # Create a mask for depth <= 1200 mm
            depth_mask = (depth <= 1200) & (depth > 0) & (~np.isnan(depth))
            mask_uint8 = depth_mask.astype(np.uint8) * 255
            filtered_image = cv2.bitwise_and(image, image, mask=mask_uint8)
            filtered_image = image.copy() #delete this line to use depth filtering
            h, w = image.shape[:2]
    
            # YOLO + ByteTrack
            results = self.model.track(
                cv2.cvtColor(filtered_image, cv2.COLOR_BGR2RGB),
                tracker="/home/lucas/catkin_ws/src/cml/follow_me/scripts/bytetrack.yaml",
                persist=True,
                classes=[0],  # class 0 = person
                verbose=False
            )
    
            depth_list = []  # (x, y, depth, track_id)
    
            if results and results[0].boxes.id is not None:
                boxes = results[0].boxes
                track_ids = boxes.id.int().cpu().tolist()
                xyxys = boxes.xyxy.cpu().numpy()
    
                for (x1, y1, x2, y2), tid in zip(xyxys, track_ids):
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    # NEW: Horizontal depth sampling at y = center_y
                    if 0 <= center_y < depth.shape[0]:
                        # Get depth values along a horizontal line in the box
                        horizontal_depths = depth[center_y, x1:x2]
                        # Filter out NaNs and invalid values
                        valid_depths = horizontal_depths[(~np.isnan(horizontal_depths)) & (horizontal_depths > 0)]
                        if valid_depths.size > 0:
                            # Pick the closest valid depth
                            closest_depth = float(np.min(valid_depths))
                            depth_list.append((center_x, center_y, closest_depth, tid))

                    # Draw detection box
                    cv2.rectangle(filtered_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(filtered_image, f"ID {tid}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    
            # Keep tracking the same person unless lost for too long
            # lost_target_frames = 0
            MAX_LOST_FRAMES = 30  # ~1 sec at 30 FPS

            if depth_list:
                if target_id is None:
                    # Lock onto the closest person initially
                    target_id = sorted(depth_list, key=lambda p: p[2])[0][3]
                    lost_target_frames = 0
                else:
                    # Check if current target is still visible
                    current_target = [d for d in depth_list if d[3] == target_id]
                    if current_target:
                        lost_target_frames = 0
                    else:
                        lost_target_frames += 1
                        rospy.loginfo(f"Lost target {target_id} for {lost_target_frames} frames")
                        if lost_target_frames > MAX_LOST_FRAMES:
                            # Only switch if lost for more than MAX_LOST_FRAMES
                            target_id = sorted(depth_list, key=lambda p: p[2])[0][3]
                            lost_target_frames = 0


    
            # Find target person in this frame
            target_data = None
            for cx, cy, dist, tid in depth_list:
                if tid == target_id:
                    target_data = (cx, cy, dist)
                    break
                
            if target_data:
                cx, cy, dist = target_data
                offset = cx - w // 2
                angle = math.atan2(offset, cy)
    
                # rospy.loginfo(f"distance = {dist}")
                # rospy.loginfo(f"angle = {angle}")
                self.distance_pub.publish(dist)
                self.angle_pub.publish(angle)
    
            cv2.imshow('Human_detect', filtered_image)
            cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('human_detect_node')
    detector = PersonDetector()
    turtlebot = TurtleBotApproach()
    turtlebot.run()

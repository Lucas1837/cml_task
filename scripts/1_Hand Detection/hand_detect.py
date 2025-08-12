#! /home/lucas/catkin_ws/src/cml/venv/bin/python3

import rospy
from std_msgs.msg import Int32, String
import cv2
import mediapipe as mp
import numpy as np
from cv_bridge import CvBridge
import pyzed.sl as sl  # Import PyZED SDK


class HandDetectionNode:
    def __init__(self):
        #Ros publisher
        rospy.init_node('hand_detection_node', anonymous=True)
        self.pointing_pub = rospy.Publisher('/pointing_hand', Int32, queue_size=10)
        self.audio_pub = rospy.Publisher('/text_to_speech',String,queue_size=10)

        # MediaPipe pose detection
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose.Pose(min_detection_confidence=0.5,
                                              min_tracking_confidence=0.5)
        self.drawing_spec = self.mp_drawing.DrawingSpec(thickness=2, circle_radius=2, color=(0, 255, 0))
        self.bridge = CvBridge()

        # Initialize ZED
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            rospy.logerr(f"Error {err} opening ZED camera. Exiting.")
            exit(1)
        self.image_zed = sl.Mat()

        # Timer control
        self.timer_started = False
        self.last_detected_hand = None  # 1 for left, 2 for right
        self.publish_once_done = False

        self.collection = [[],[]]

    def calculate_angle(self, landmark1, landmark2, landmark3):
        a = np.array([landmark1.x, landmark1.y])
        b = np.array([landmark2.x, landmark2.y])
        c = np.array([landmark3.x, landmark3.y])
        ba = a - b
        bc = c - b
        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)
        return np.degrees(angle)

    def is_landmark_visible(self, landmark):
        return landmark.visibility > 0.5

    def start_shutdown_timer(self):
        if not self.timer_started:
            self.timer_started = True
            rospy.loginfo("Arm detected, starting 3-second shutdown timer...")
            rospy.Timer(rospy.Duration(3), self.publish_and_shutdown, oneshot=True)

    def run(self):
        audio1 = False
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
                if not audio1:
                    self.audio_pub.publish("Please point to the bag you want me to carry and stay still for 3 seconds")
                    audio1 = True
                self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
                cv_image = self.image_zed.get_data()
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
                # cv_image = cv2.flip(cv_image, 0)
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

                pose_results = self.mp_pose.process(rgb_image)

                if pose_results.pose_landmarks:
                    self.mp_drawing.draw_landmarks(
                        cv_image, pose_results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)

                    right_elbow = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_ELBOW]
                    right_shoulder = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]
                    right_hip = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_HIP]
                    right_wrist = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_WRIST]

                    left_elbow = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_ELBOW]
                    left_shoulder = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
                    left_hip = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_HIP]
                    left_wrist = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_WRIST]

                    # Right arm pointing (actually left due to flip)
                    if all(self.is_landmark_visible(lm) for lm in
                           [right_elbow, right_shoulder, right_hip, left_elbow, left_wrist]):
                        right_angle = self.calculate_angle(right_elbow, right_shoulder, right_hip)
                        left_elbow_bend_angle = self.calculate_angle(left_shoulder, left_elbow, left_wrist)
                        if right_angle > 18 and left_elbow_bend_angle > 155:
                            cv2.putText(cv_image, "Right Arm Pointing Detected", (50, 50),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                            self.last_detected_hand = 2
                            self.start_shutdown_timer()


                    if all(self.is_landmark_visible(lm) for lm in
                           [left_elbow, left_shoulder, left_hip, right_elbow, right_wrist]):
                        left_angle = self.calculate_angle(left_elbow, left_shoulder, left_hip)
                        right_elbow_bend_angle = self.calculate_angle(right_shoulder, right_elbow, right_wrist)
                        if left_angle > 18 and right_elbow_bend_angle > 155:
                            cv2.putText(cv_image, "Left Arm Pointing Detected", (50, 100),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            self.last_detected_hand = 1
                            self.start_shutdown_timer()

                    if self.last_detected_hand == 1:
                        self.collection[0].append(1)
                    
                    elif self.last_detected_hand == 2:
                        self.collection[1].append(2)

                cv2.imshow('Pose Detection', cv_image)
                cv2.waitKey(1)

            rate.sleep()

    def publish_and_shutdown(self, event=None):
        if not self.publish_once_done and self.last_detected_hand is not None:

            if len(self.collection[0])>len(self.collection[1]):
                self.last_detected_hand = 1
            elif len(self.collection[0])<len(self.collection[1]):
                self.last_detected_hand =2
            #TTS
            if self.last_detected_hand ==1:
                self.audio_pub.publish("You have choosen the Left bag")
            elif self.last_detected_hand == 2:
                self.audio_pub.publish("You have choosen the Right bag")

            rospy.loginfo(f"Publishing final pointing hand: {self.last_detected_hand}")
            self.pointing_pub.publish(self.last_detected_hand)
            self.publish_once_done = True

        rospy.loginfo("Shutting down after publishing...")
        cv2.destroyAllWindows()
        # self.zed.close()
        rospy.signal_shutdown("Time limit reached")


if __name__ == '__main__':
    node = HandDetectionNode()
    node.run()

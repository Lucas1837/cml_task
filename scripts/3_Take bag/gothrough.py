#! /home/lucas/catkin_ws/src/cml/venv/bin/python3
import rospy
import math
import subprocess
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray

class GoThrough:
    def __init__(self):
        rospy.init_node("gothrough", anonymous=True)

        # Variables
        self.target_distance_mm = None
        self.arm_ready = False
        self.start_pose = None
        self.reached_target = False
        self.bag_loc_received = False  # Only store once
        self.script_started = False    # To avoid multiple calls

        # Subscribers
        rospy.Subscriber("/bag_loc", Float32MultiArray, self.bag_loc_callback)
        rospy.Subscriber("/arm_ready", Bool, self.arm_ready_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Publisher
        self.cmd_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)

        rospy.loginfo("gothrough node started.")
        rospy.spin()

    def bag_loc_callback(self, msg):
        if not self.bag_loc_received and len(msg.data) >= 1:
            self.target_distance_mm = msg.data[0] - 30 # First element = depth in mm
            self.bag_loc_received = True
            rospy.loginfo(f"Stored first target distance: {self.target_distance_mm} mm")

    def arm_ready_callback(self, msg):
        self.arm_ready = msg.data
        rospy.loginfo(f"Arm ready status: {self.arm_ready}")

    def odom_callback(self, msg):

        if not self.arm_ready or self.target_distance_mm is None or self.reached_target:
            return

        position = msg.pose.pose.position

        if self.start_pose is None:
            self.start_pose = position
            rospy.loginfo("Starting position recorded.")
            return  # Wait for next callback

        # Calculate traveled distance in meters
        dx = position.x - self.start_pose.x
        dy = position.y - self.start_pose.y
        distance_m = math.sqrt(dx*dx + dy*dy)
        target_distance_m = (self.target_distance_mm - 100) / 1000.0

        twist = Twist()

        if distance_m < target_distance_m:
            twist.linear.x = 0.3
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

            if not self.script_started:
                subprocess.Popen(["rosrun", "cml", "pickupTall.py"])
                self.script_started = True

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.reached_target = True
            rospy.loginfo("Reached target distance.")

if __name__ == "__main__":
    try:
        GoThrough()
    except rospy.ROSInterruptException:
        pass

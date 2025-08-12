#! /home/lucas/catkin_ws/src/cml/venv/bin/python3

import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

class TurtleBotApproach:
    def __init__(self):
        rospy.init_node('turtlebot_control_node')
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reach_distance_pub = rospy.Publisher('/reach_distance', Bool, queue_size=10)
        rospy.Subscriber('/bag_distance', Float32, self.distance_callback)
        rospy.Subscriber('/bag_angle', Float32, self.angle_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.target_distance = 0.7 # meters
        self.target_angle = 0       # radian

        self.max_linear_vel = 0.2
        self.max_angular_vel = 0.4

        self.cmd_vel = Twist()
        self.distance_to_bag = None
        self.angle_to_bag = None
        self.has_reached_distance = False

        # Yaw tracking
        self.initial_yaw = None
        self.final_yaw = None
        self.current_yaw = None

    def odom_callback(self, msg):
        """Extract yaw from odometry quaternion."""
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.current_yaw = yaw

        # Record initial yaw once
        if self.initial_yaw is None:
            self.initial_yaw = yaw

    def distance_callback(self, msg):
        self.distance_to_bag = msg.data / 1000  # mm to meters
        self.update_velocity()

    def angle_callback(self, msg):
        self.angle_to_bag = msg.data
        self.update_velocity()

    def update_velocity(self):
        if self.distance_to_bag is None or self.angle_to_bag is None or self.current_yaw is None:
            rospy.loginfo("Waiting for distance, angle, and yaw measurements...")
            return

        error_dist = self.distance_to_bag - self.target_distance
        error_ang = self.angle_to_bag - self.target_angle

        # Stop if bag is within distance threshold
        if self.distance_to_bag < self.target_distance:
            if not self.has_reached_distance:
                rospy.loginfo("Target distance reached. Stopping robot.")
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
                self.vel_pub.publish(self.cmd_vel)

                # Record final yaw
                self.final_yaw = self.current_yaw
                yaw_diff = self.final_yaw - self.initial_yaw

                rospy.loginfo(f"Initial yaw: {self.initial_yaw:.3f}, Final yaw: {self.final_yaw:.3f}, Diff: {yaw_diff:.3f}")

                # Rotate back to compensate
                self.rotate_back(yaw_diff)

                self.reach_distance_pub.publish(True)
                self.has_reached_distance = True
                rospy.signal_shutdown("Robo_bag shutting down")
            return

        # Move toward target
        self.cmd_vel.linear.x = self.max_linear_vel
        if error_ang > 0.3 and self.angle_to_bag != 0:
            self.cmd_vel.angular.z = self.max_angular_vel
        elif error_ang < -0.3 and self.angle_to_bag != 0:
            self.cmd_vel.angular.z = -self.max_angular_vel
        else:
            self.cmd_vel.angular.z = 0

        self.vel_pub.publish(self.cmd_vel)

    def rotate_back(self, yaw_diff):
        """Rotate robot in opposite direction of yaw_diff."""
        target_angle = -yaw_diff  # Opposite direction
        rospy.loginfo(f"Rotating back by {target_angle:.3f} rad")

        # Normalize angle to [-pi, pi]
        target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))

        # Record starting yaw
        start_yaw = self.current_yaw
        turned_angle = 0

        rate = rospy.Rate(10)
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = self.max_angular_vel if target_angle > 0 else -self.max_angular_vel

        while abs(turned_angle) < abs(target_angle) and not rospy.is_shutdown():
            self.vel_pub.publish(self.cmd_vel)
            turned_angle = self.current_yaw - start_yaw
            turned_angle = math.atan2(math.sin(turned_angle), math.cos(turned_angle))
            rate.sleep()

        # Stop after turning
        self.cmd_vel.angular.z = 0
        self.vel_pub.publish(self.cmd_vel)
        rospy.loginfo("Rotation back complete.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        turtlebot_approach = TurtleBotApproach()
        turtlebot_approach.run()
    except rospy.ROSInterruptException:
        pass

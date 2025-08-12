#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Int32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class RotateKobuki:
    def __init__(self):
        rospy.init_node("rotate_kobuki")

        self.cmd_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        self.rotated_pub = rospy.Publisher("/rotated", Bool, queue_size=10)
        rospy.Subscriber("/pointing_hand", Int32, self.command_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        

        self.current_yaw = None
        self.target_yaw = None
        self.rotating = False
        self.direction = 0  # +1 for CCW, -1 for CW
        self.angular_speed = 1  # rad/s (~17°/s)

    def normalize_angle(self, angle):
        """Keep angle in range [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.current_yaw = yaw

        if self.rotating and self.target_yaw is not None:
            angle_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(angle_error) < math.radians(2):  # within 2° tolerance
                self.stop_rotation()

    def command_callback(self, msg):
        if self.current_yaw is None or self.rotating:
            return  # ignore until odom ready or already rotating

        if msg.data == 1:
            self.direction = -1   # CCW
        elif msg.data == 2:
            self.direction = 1   # CW
        else:
            return  # invalid command

        # Calculate target yaw
        self.target_yaw = self.normalize_angle(
            self.current_yaw + self.direction * math.radians(25 )
        )
        self.start_rotation()

    def start_rotation(self):
        rospy.loginfo(f"Rotating {'CCW' if self.direction == 1 else 'CW'} to {math.degrees(self.target_yaw):.2f}°")
        self.rotating = True
        twist = Twist()
        twist.angular.z = self.direction * self.angular_speed
        self.cmd_pub.publish(twist)

    def stop_rotation(self):
        rospy.loginfo("Rotation complete.")
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.rotating = False
        self.target_yaw = None
        self.direction = 0
        self.rotated_pub.publish(True)
        rospy.signal_shutdown("Rotation completed")

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.rotating:
                twist = Twist()
                twist.angular.z = self.direction * self.angular_speed
                self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    node = RotateKobuki()
    node.spin()

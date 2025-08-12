#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import subprocess
class KobukiHorizontalMover:
    def __init__(self):
        rospy.init_node("horizontal_mover", anonymous=True)

        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.aligned_pub = rospy.Publisher('/aligned', Bool, queue_size=10)
        rospy.Subscriber('/reach_distance',Bool,self.reachcallback)
        rospy.Subscriber('/horizontal_dist', Float32, self.dist_callback,queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_yaw = 0.0
        self.current_pos = None
        self.executed = False
        self.rate = rospy.Rate(10)
        self.reached = False
        self.last_horz = None

    def reachcallback(self,msg):
        self.reached = msg.data

    def odom_callback(self, msg):
        # Update yaw
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        # Update position
        pos = msg.pose.pose.position
        self.current_pos = (pos.x, pos.y)
        if self.executed:
                rospy.sleep(1)
                subprocess.Popen(['rosrun', 'cml', 'gothrough.py'])
                subprocess.Popen(['rosrun', 'cml', 'armIK.py'])
                rospy.signal_shutdown("Aligned Task completed")


    def dist_callback(self, msg):
        if not self.executed and self.reached: 
            rospy.loginfo(f"reach:{self.reached}")
            horiz_mm = msg.data 
            rospy.loginfo(f"Received horizontal distance: {horiz_mm:.1f} mm")
            if self.last_horz is not None and abs(self.last_horz - horiz_mm) > self.last_horz:
                rospy.loginfo("Horizontal distance is not stable, skipping execution.")
                return
            if abs(horiz_mm) > 40:
                self.execute_move(horiz_mm)
                self.last_horz = horiz_mm
            else:
                self.aligned_pub.publish(True)
                self.executed = True

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def rotate_to_target_yaw(self, target_yaw, speed=0.5, tol_deg=2.0):
        tol = math.radians(tol_deg)
        twist = Twist()
        while not rospy.is_shutdown():
            error = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(error) <= tol:
                break
            twist.angular.z = speed if error > 0 else -speed
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            self.rate.sleep()
        self.cmd_pub.publish(Twist())  # stop

    def move_forward(self, distance_m, speed=0.2):
        if self.current_pos is None:
            rospy.logwarn("Position unknown — can't move forward.")
            return
        start_pos = self.current_pos
        twist = Twist()
        twist.linear.x = speed
        while not rospy.is_shutdown():
            dx = self.current_pos[0] - start_pos[0]
            dy = self.current_pos[1] - start_pos[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist >= abs(distance_m):
                break
            self.cmd_pub.publish(twist)
            self.rate.sleep()
        self.cmd_pub.publish(Twist())  # stop

    def execute_move(self, horiz_mm):
        # Decide turn direction
            if horiz_mm < 0:
                rospy.loginfo("Turning CounterCLOCKWISE 90°")
                target_yaw_1 = self.normalize_angle(self.current_yaw - math.pi/2)
                clockwise = True
            else:
                rospy.loginfo("Turning CLOCKWISE 90°")
                target_yaw_1 = self.normalize_angle(self.current_yaw + math.pi/2)
                clockwise = False

            # First turn
            self.rotate_to_target_yaw(target_yaw_1, speed=0.7)

            # Move forward by |distance|
            self.move_forward(abs(horiz_mm) / 1000.0, speed=0.2)

            # Turn back to original
            if clockwise:
                target_yaw_2 = self.normalize_angle(target_yaw_1 + math.pi/2)
            else:
                target_yaw_2 = self.normalize_angle(target_yaw_1 - math.pi/2)
            rospy.loginfo("Turning back to front")
            self.rotate_to_target_yaw(target_yaw_2, speed=0.7)
            rospy.sleep(0.5)    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = KobukiHorizontalMover()
        node.run()
    except rospy.ROSInterruptException:
        pass

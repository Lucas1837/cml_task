#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
import time

class TurtleBotApproach:
    def __init__(self):
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reach_distance_pub = rospy.Publisher('/reach_distance', Bool, queue_size=10)

        rospy.Subscriber('/person_distance', Float32, self.distance_callback)
        rospy.Subscriber('/person_angle', Float32, self.angle_callback)

        # Settings
        self.target_distance = 0.4   # meters
        self.target_angle = 0        # radians
        self.stop_tolerance = 0.05   # meters
        self.timeout_sec = 0.5       # stop if no new messages after this time

        # State variables
        self.distance_to_person = None
        self.angle_to_person = None
        self.last_distance_time = None
        self.last_angle_time = None
        self.has_reached_distance = False
        self.last_dist = None

    def distance_callback(self, msg):
        self.distance_to_person = msg.data / 1000.0  # mm → m
        self.last_distance_time = time.time()

    def angle_callback(self, msg):
        self.angle_to_person = msg.data
        self.last_angle_time = time.time()

    def should_stop_due_to_timeout(self):
        now = time.time()
        if (self.last_distance_time is None) or (self.last_angle_time is None):
            return True
        if (now - self.last_distance_time > self.timeout_sec) or \
           (now - self.last_angle_time > self.timeout_sec):
            return True
        return False

    def update_velocity(self):
        cmd_vel = Twist()

        # Safety stop if no fresh data
        if self.should_stop_due_to_timeout():

            self.vel_pub.publish(cmd_vel)  # all zeros
            return

        error_dist = self.distance_to_person - self.target_distance
        error_ang = self.angle_to_person - self.target_angle

        # Stop linear motion if within target distance
        if abs(error_dist) <= self.stop_tolerance:
            cmd_vel.linear.x = 0.0
            self.has_reached_distance = True
            self.reach_distance_pub.publish(True)
        else:
            self.has_reached_distance = False
            self.reach_distance_pub.publish(False)

            # Variable speed based on distance
            
            if self.distance_to_person > self.target_distance + 1.9:
                cmd_vel.linear.x = 2
            elif self.distance_to_person > self.target_distance + 1.8:
                cmd_vel.linear.x = 1.9
            elif self.distance_to_person > self.target_distance + 1.7:
                cmd_vel.linear.x = 1.8
            elif self.distance_to_person > self.target_distance + 1.6:
                cmd_vel.linear.x = 1.7
            elif self.distance_to_person > self.target_distance + 1.5:
                cmd_vel.linear.x = 1.6
            elif self.distance_to_person > self.target_distance + 1.4:
                cmd_vel.linear.x = 1.5
            elif self.distance_to_person > self.target_distance + 1.3:
                cmd_vel.linear.x = 1.4
            elif self.distance_to_person > self.target_distance + 1.2:
                cmd_vel.linear.x = 1.3
            elif self.distance_to_person > self.target_distance + 1.1:
                cmd_vel.linear.x = 1.2
            elif self.distance_to_person > self.target_distance + 1.0:
                cmd_vel.linear.x = 1.1
            elif self.distance_to_person > self.target_distance + 0.9:
                cmd_vel.linear.x = 1
            elif self.distance_to_person > self.target_distance + 0.8:
                cmd_vel.linear.x = 0.9
            elif self.distance_to_person > self.target_distance + 0.7:
                cmd_vel.linear.x = 0.8
            elif self.distance_to_person > self.target_distance + 0.6:
                cmd_vel.linear.x = 0.7
            elif self.distance_to_person > self.target_distance + 0.5:
                cmd_vel.linear.x = 0.6
            elif self.distance_to_person > self.target_distance + 0.4:
                cmd_vel.linear.x = 0.5
            elif self.distance_to_person > self.target_distance + 0.3:
                cmd_vel.linear.x = 0.4
            elif self.distance_to_person > self.target_distance + 0.2:
                cmd_vel.linear.x = 0.3
            elif self.distance_to_person > self.target_distance + 0.1:
                cmd_vel.linear.x = 0.2
            elif self.distance_to_person > self.target_distance + 0.05:
                cmd_vel.linear.x = 0.1                
            else:
                cmd_vel.linear.x = 0

        # Angular velocity adjustment
        # rospy.loginfo(f"Angular Error: {error_ang}")
        if error_ang > 0.8:
            cmd_vel.angular.z = -1.6
        elif error_ang > 0.6:
            cmd_vel.angular.z = -1.3
        elif error_ang > 0.2:
            cmd_vel.angular.z = -0.8
        elif error_ang < -0.8:
            cmd_vel.angular.z = 1.6
        elif error_ang < -0.6:
            cmd_vel.angular.z = 1.3
        elif error_ang < -0.2:
            cmd_vel.angular.z = 0.8
        else:
            cmd_vel.angular.z = 0.0

        self.vel_pub.publish(cmd_vel)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_velocity()
            rate.sleep()

if __name__ == "__main__":
    bot = TurtleBotApproach()
    bot.run()

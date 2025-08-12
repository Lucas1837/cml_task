#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist
import time

class TurtleBotApproach:
    def __init__(self):

        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reach_distance_pub = rospy.Publisher('/reach_distance', Bool, queue_size=10)

        rospy.Subscriber('/person_distance', Float32, self.distance_callback)
        rospy.Subscriber('/person_angle', Float32, self.angle_callback)

        # Settings
        self.target_distance = 0.4  # meters
        self.target_angle = 0       # radians
        self.stop_tolerance = 0.05   # meters
        # State variables
        self.distance_to_person = None
        self.angle_to_person = None
        self.has_reached_distance = False
        self.timer_start = False
        self.audio = False
        self.start = 0

    def distance_callback(self, msg):
        self.distance_to_person = msg.data / 1000.0  # mm → m

    def angle_callback(self, msg):
        self.angle_to_person = msg.data

    def update_velocity(self):
        # Only run if there are both measurements
        if self.distance_to_person is None or self.angle_to_person is None:
            return

        cmd_vel = Twist()
        error_dist = self.distance_to_person - self.target_distance
        error_ang = self.angle_to_person - self.target_angle

        # Stop linear motion if within target distance
        if abs(error_dist) <= self.stop_tolerance:
            cmd_vel.linear.x = 0.0
            self.has_reached_distance = True
            self.reach_distance_pub.publish(True)
        else:
            # Resume forward motion if person moved away
            self.has_reached_distance = False
            self.reach_distance_pub.publish(False)

            # Variable speed based on distance
            if self.distance_to_person > self.target_distance + 1.5:
                cmd_vel.linear.x = 2.0
            elif self.distance_to_person > self.target_distance + 1.3:
                cmd_vel.linear.x = 1.8
            elif self.distance_to_person > self.target_distance + 1.1:
                cmd_vel.linear.x = 1.6
            elif self.distance_to_person > self.target_distance + 0.9:
                cmd_vel.linear.x = 1.4
            elif self.distance_to_person > self.target_distance + 0.7:
                cmd_vel.linear.x = 0.8
            elif self.distance_to_person > self.target_distance + 0.5:
                cmd_vel.linear.x = 0.6
            elif self.distance_to_person > self.target_distance + 0.3:
                cmd_vel.linear.x = 0.4
            elif self.distance_to_person > self.target_distance + 0.1:
                cmd_vel.linear.x = 0.2
            else:
                cmd_vel.linear.x = 0

        # Angular velocity adjustment
        if error_ang > 0.8:
            cmd_vel.angular.z = -1.6
        elif error_ang > 0.6:
            cmd_vel.angular.z = -1.1
        elif error_ang > 0.2:
            cmd_vel.angular.z = -0.6
        elif error_ang < -0.8:
            cmd_vel.angular.z = 1.6
        elif error_ang < -0.6:
            cmd_vel.angular.z = 1.1
        elif error_ang < -0.2:
            cmd_vel.angular.z = 0.6
        else:
            cmd_vel.angular.z = 0.0

        # Publish combined motion (linear + angular together)
        self.vel_pub.publish(cmd_vel)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_velocity()
            rate.sleep()

if __name__ == "__main__":
    bot = TurtleBotApproach()
    bot.run()

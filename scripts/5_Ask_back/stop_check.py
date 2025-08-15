#!/home/lucas/catkin_ws/src/edurobot/myenv/bin/python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import subprocess 

class RobotStopDetector:
    def __init__(self):
        rospy.init_node('movement_check', anonymous=True)

        self.ask_pub = rospy.Publisher("/ask_stop_follow", String, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/voice_trigger", String, self.voice_callback)

        self.last_movement_time = rospy.Time.now()
        self.is_stopped = False
        self.waiting_for_movement_after_no = False
        self.user_wants_stop = False
        self.stopped_ask_time = None
        self.vel_threshold = 0.01

    def odom_callback(self, msg):
        vel = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        now = rospy.Time.now()

        if vel > self.vel_threshold:
            # Robot moving again → cancel any asking loop
            if self.is_stopped or self.waiting_for_movement_after_no:
                rospy.loginfo("▶️ Robot started moving again → cancel asking state.")
                self.ask_pub.publish("cancel_ask")
            self.last_movement_time = now
            self.is_stopped = False
            self.waiting_for_movement_after_no = False
            return

        # Robot is stationary
        if (now - self.last_movement_time).to_sec() > 5.0:
            if not self.is_stopped:
                rospy.loginfo("⏹️ Robot stopped for 5s, asking user...")
                self.ask_pub.publish("ask")
                self.is_stopped = True
                self.stopped_ask_time = now
            else:
                # After NO response, still not moving → ask again after 10s
                if self.waiting_for_movement_after_no and \
                   (now - self.stopped_ask_time).to_sec() > 10.0:
                    rospy.loginfo("⏹️ Robot still not moving after NO, asking again...")
                    self.ask_pub.publish("ask")
                    self.stopped_ask_time = now

    def voice_callback(self, msg):
        if msg.data == "stop_following":
            rospy.loginfo("🛑 User said YES → stop following and exit")
            self.user_wants_stop = True
            subprocess.Popen(['rosrun','cml','drop.py'])
            subprocess.Popen(['rosnode','kill','skeleton_pose_node'])
            subprocess.Popen(['rosnode','kill','handle_detection_node'])
            rospy.signal_shutdown("User requested stop")
        elif msg.data == "continue_following":
            rospy.loginfo("🔄 User said NO → will check movement again")
            self.waiting_for_movement_after_no = True
            self.stopped_ask_time = rospy.Time.now()

if __name__ == "__main__":
    try:
        RobotStopDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

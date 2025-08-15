#! /home/lucas/catkin_ws/src/cml/venv/bin/python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import Bool
activate_pub = rospy.Publisher('/activate_model', Bool, queue_size=10)
    
def send_waypoints(client, waypoint):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose = waypoint

    rospy.loginfo(f"Sending waypoint {waypoint}")
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(40.0))  # Wait for a max of 10 seconds
    rospy.loginfo(f"Waypoint {waypoint} reached or timeout reached")

if __name__ == '__main__':
    rospy.init_node('navigation_with_waypoints')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Define multiple waypoints
    waypoint = Pose(position=Point(-104.54584, -100.74004, 0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))

    # Send the waypoints to the robot
    send_waypoints(client, waypoint)

    rospy.loginfo("All waypoints processed")



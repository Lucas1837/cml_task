#! /home/lucas/catkin_ws/src/cml/venv/bin/python3
import serial
import time
import math
import rospy
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import subprocess

# ========== Global Variables ==========
current_yaw = 0.0
start_yaw = 0.0
current_pos = None

# ========== ROS Setup ==========
rospy.init_node('drop_and_retreat')
cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
audio_pub = rospy.Publisher('/text_to_speech',String,queue_size=10)
def odom_callback(msg):
    global current_yaw, current_pos
    q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    current_yaw = yaw
    pos = msg.pose.pose.position
    current_pos = (pos.x, pos.y)

rospy.Subscriber('/odom', Odometry, odom_callback)
rospy.sleep(0.5)  # Allow odometry to initialize

# ========== Serial Port Setup ==========
port = "/dev/ttyUSB0"
ser = serial.Serial(port, 1000000)
time.sleep(0.1)
print("🔌 Serial port open:", ser.is_open)

# Constants
start = [0xFF, 0xFF]
length = 5
instruction = 3
address_position = 30
address_speed = 32
default_speed = 128

# ---------- Servo Utility Functions ----------
def angle_to_raw(angle):
    value = math.floor(angle * 1023 / 300)
    return value % 256, value // 256

def speed_to_raw(speed):
    return speed % 256, speed // 256

def checksum(id, length, instruction, address, valL, valH):
    total = id + length + instruction + address + valL + valH
    return 255 - (total % 256)

def send_packet(id, address, valL, valH):
    chksum = checksum(id, length, instruction, address, valL, valH)
    packet = start + [id, length, instruction, address, valL, valH, chksum]
    ser.write(bytes(packet))

def set_speed(id, speed=default_speed):
    valL, valH = speed_to_raw(speed)
    send_packet(id, address_speed, valL, valH)
    print(f"⚙️ Speed set to {speed} for ID {id}")
    time.sleep(0.01)

def write_position(id, angle):
    valL, valH = angle_to_raw(angle)
    send_packet(id, address_position, valL, valH)
    print(f"✅ ID {id} moved to {angle}°")

# ---------- Kobuki Movement ----------
def move_distance(distance_m, speed=0.4):
    """Moves the robot a given distance in meters (negative for backward)."""
    global current_pos
    if current_pos is None:
        rospy.logwarn("No odometry data — can't move")
        return
    start_pos = current_pos
    twist = Twist()
    twist.linear.x = speed
    while not rospy.is_shutdown():
        dx = current_pos[0] - start_pos[0]
        dy = current_pos[1] - start_pos[1]
        traveled = math.sqrt(dx**2 + dy**2)
        if traveled >= abs(distance_m):
            break
        cmd_pub.publish(twist)
        rospy.sleep(0.05)
    cmd_pub.publish(Twist())

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def rotate_angle(angle_rad, speed=0.5):
    """Rotates the robot a given angle in radians."""
    global current_yaw
    start_yaw = current_yaw
    target_yaw = normalize_angle(start_yaw + angle_rad)
    twist = Twist()
    while not rospy.is_shutdown():
        error = normalize_angle(target_yaw - current_yaw)
        if abs(error) < math.radians(2):  # 2° tolerance
            break
        twist.angular.z = speed if error > 0 else -speed
        cmd_pub.publish(twist)
        rospy.sleep(0.05)
    cmd_pub.publish(Twist())

# ---------- Main Sequence ----------
try:
    audio_pub.publish("Help me drop the bag")
    ids = [1, 3, 6]
    for i in ids:
        set_speed(i)

    # Step 1: Move arm to drop bag
    write_position(1, 230)
    write_position(6, 60)
    write_position(3, 130)
    rospy.sleep(2)  # wait for arm movement

    # Step 2: Move backward 30 cm
    move_distance(-0.3, speed=-0.15)

    # Step 3: Rotate 180°
    rotate_angle(math.pi, speed=0.5)

    subprocess.Popen(['rosnode','cml','queue_up_node'])

except KeyboardInterrupt:
    print("❌ Interrupted")

finally:
    ser.close()
    print("🔌 Serial port closed.")

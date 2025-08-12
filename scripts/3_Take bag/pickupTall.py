#! /home/lucas/catkin_ws/src/cml/venv/bin/python3

import serial
import time
import math
import subprocess
import rospy
from std_msgs.msg import String

rospy.init_node('pickup')

# Initialize the serial port
port = "/dev/ttyUSB0"
ser = serial.Serial(port, 1000000)
ser.close()
ser.open()
print("Serial port open:", ser.isOpen())

# ---------- Constants ----------
start = [0xFF, 0xFF]
length = 5
instruction = 3
address_position = 30
address_speed = 32
default_speed = 128

# ---------- Utility Functions ----------

def GoalPositionValH(angle):
    value = math.floor(angle * 3.41)
    return value // 256

def GoalPositionValL(angle):
    value = math.floor(angle * 3.41)
    return value % 256

def SpeedValH(speed):
    return speed // 256

def SpeedValL(speed):
    return speed % 256

def checksum(id, length, instruction, address, valL, valH):
    total = id + length + instruction + address + valL + valH
    return 255 - (total % 256)

def set_speed(id, speed=default_speed):
    SpeedH = SpeedValH(speed)
    SpeedL = SpeedValL(speed)
    chksum = checksum(id, length, instruction, address_speed, SpeedL, SpeedH)
    packet = start + [id, length, instruction, address_speed, SpeedL, SpeedH, chksum]
    ser.write(bytes(packet))
    print(f"🚀 Speed set to {speed} for ID {id}")

def write_position(id, angle):
    DegH = GoalPositionValH(angle)
    DegL = GoalPositionValL(angle)
    chksum = checksum(id, length, instruction, address_position, DegL, DegH)
    packet = start + [id, length, instruction, address_position, DegL, DegH, chksum]
    ser.write(bytes(packet))
    print(f"✅ ID {id} moved to {angle}°")

# ---------- Main Execution ----------
try:
    # Set all speeds first
    for sid in [1, 6, 3]:
        set_speed(sid)
        time.sleep(0.05)

    # x# Step 1: Move ID 1 to 150°
    # write_position(1, 220)
    # time.sleep(1.2)  # wait for completion

    # # Step 2: Move ID 6 to 110°
    # write_position(6, 110)
    # time.sleep(1.2)

    # Step 3: Move all (ID 1 → 290°, ID 6 → 100°, ID 3 → 190°)
    write_position(1, 230)
    time.sleep(2)
    write_position(6, 80)
    write_position(3, 170)
    time.sleep(3.0)
    subprocess.Popen(['rosrun','cml_task','skeleton_pose.py'])
    subprocess.Popen(['roslaunch','cml_task','navi.launch'])
    subprocess.Popen(['rosnode','kill','handle_detection_node'])

except KeyboardInterrupt:
    print("❌ Interrupted by user")

finally:
    ser.close()
    print("Serial port closed.")

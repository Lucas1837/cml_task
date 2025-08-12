#! /home/lucas/catkin_ws/src/cml/venv/bin/python3
import serial
import time
import math
import os
import rospy
from IK import inverse_kinematics
from std_msgs.msg import Float32MultiArray, Bool

class InverseKinematicsController:
    def __init__(self):
        # Serial setup
        port = "/dev/ttyUSB0"
        self.ser = serial.Serial(port, 1000000)
        self.ser.close()
        self.ser.open()
        rospy.loginfo(f"Serial port open: {self.ser.isOpen()}")

        # Constants
        self.start = [255, 255]
        self.length = 5
        self.Write_instruction = 3
        self.address_position = 30
        self.address_speed = 32
        self.default_speed = 128
        self.servo_order = [3, 6, 1]
        self.count=0
        # Flags
        self.flag = True

        # ROS setup
        rospy.init_node("inverse_kinematics_controller", anonymous=True)
        self.ready_pub = rospy.Publisher('/arm_ready', Bool, queue_size=10)
        rospy.Subscriber("/bag_loc", Float32MultiArray, self.coordinate_callback, queue_size=1)

    # ---------- Servo Utility Functions ----------
    def GoalPositionValH(self, angle):
        value = math.floor(angle * 3.41)
        return value // 256

    def GoalPositionValL(self, angle):
        value = math.floor(angle * 3.41)
        return value % 256

    def SpeedValH(self, speed):
        return speed // 256

    def SpeedValL(self, speed):
        return speed % 256

    def checksum(self, id, length, instruction, address, valL, valH):
        total = id + length + instruction + address + valL + valH
        return 255 - (total % 256)

    def write_position(self, angle, servo_id):
        DegH = self.GoalPositionValH(angle)
        DegL = self.GoalPositionValL(angle)
        chksum = self.checksum(servo_id, self.length, self.Write_instruction, self.address_position, DegL, DegH)
        packet = self.start + [servo_id, self.length, self.Write_instruction, self.address_position, DegL, DegH, chksum]
        self.ser.write(bytes(packet))
        rospy.loginfo(f"✅ ID {servo_id} moved to {angle}°")

    def set_speed(self, servo_id, speed=None):
        if speed is None:
            speed = self.default_speed
        SpeedH = self.SpeedValH(speed)
        SpeedL = self.SpeedValL(speed)
        chksum = self.checksum(servo_id, self.length, self.Write_instruction, self.address_speed, SpeedL, SpeedH)
        packet = self.start + [servo_id, self.length, self.Write_instruction, self.address_speed, SpeedL, SpeedH, chksum]
        self.ser.write(bytes(packet))
        rospy.loginfo(f"🚀 Speed set to {speed} for ID {servo_id}")

    def transform_angles(self, raw_angles):
        theta1, theta2, theta3 = raw_angles
        a3 = 240 - theta3
        a2 = theta2 + 150
        a1 = theta1 + 60 

        if not (20 <= a1 <= 300): return None
        if not (0 <= a2 <= 300): return None
        if a3 <= 150 or a3 >= 295: return None

        return [a1, a2, a3]

    # ---------- ROS Callback ----------
    def coordinate_callback(self, msg):
        if not self.flag:
            return

        camera_handle_distance = 600
        camera_height = 230
        length_tip_arm = 165
        # x = msg.data[0] - length_tip_arm
        x = 350 - length_tip_arm # set a value of x with a variety of achieveable y    
        y = msg.data[1] + camera_height +65

        angle_sets = inverse_kinematics(x, y)

        if not angle_sets:
            rospy.logwarn("❌ No inverse kinematics solution found.")
            return

        chosen_angles = None
        for angles in angle_sets:
            transformed = self.transform_angles(angles)
            if transformed:
                chosen_angles = transformed
                break

        if not chosen_angles:
            rospy.logwarn("❌ No valid servo angles within range.")
            return

        servo_angles_map = dict(zip([3, 6, 1], chosen_angles))
        for sid in self.servo_order:
            self.set_speed(sid)
            time.sleep(0.05)
            angle = servo_angles_map[sid]
            self.write_position(angle, sid)
            time.sleep(0.3)
        self.count+= 1
        if self.count ==2:
            self.flag = False
        self.ready_pub.publish(True)

    # ---------- Main loop ----------
    def run(self):
        rospy.loginfo("🟢 Ready to receive coordinates...")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        except KeyboardInterrupt:
            rospy.logwarn("🔴 Interrupted by user")
        finally:
            self.ser.close()
            rospy.loginfo("🔌 Serial port closed.")


if __name__ == "__main__":
    controller = InverseKinematicsController()
    controller.run()

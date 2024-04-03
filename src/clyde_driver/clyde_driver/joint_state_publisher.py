#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import re
import math

class JointAngleReader(Node):
    def __init__(self):
        super().__init__('joint_angle_reader')
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        try:
            self.serial_port = serial.Serial('/dev/ttyS0', 115200, timeout=1)  # Added error handling for serial port
        except serial.SerialException as e:
            self.get_logger().fatal(f"Failed to open serial port: {e}")
            self.destroy_node()
            return
        self.timer = self.create_timer(0.2, self.query_and_publish_joint_angles)
        self.last_joint_angles = []

        #Correct way to declare a parameter with a default value
        default_joint_names = [
            'neck_joint', 'shlfs_joint', 'shrfs_joint', 'shrrs_joint', 'shlrs_joint',
            'shlft_joint', 'shrft_joint', 'shrrt_joint', 'shlrt_joint']
        self.declare_parameter('joint_names', default_joint_names)
        self.joint_names = self.get_parameter('joint_names').value

    def query_and_publish_joint_angles(self):
        self.serial_port.write(b'j\n')
        start_time = time.time()
        while (time.time() - start_time) < 2.0:  # Improved serial read timing with timeout
            if self.serial_port.in_waiting:
                # Proceed with reading and processing data
                self.serial_port.readline()  # Read and ignore the first line (assuming it's headers)
                angle_line = self.serial_port.readline().decode().strip()
                joint_angles = self.extract_joint_angles(angle_line)
                if joint_angles is not None:
                    if joint_angles != self.last_joint_angles:  # Check to avoid redundant publishing
                        self.publish_joint_angles(joint_angles)
                    self.last_joint_angles = joint_angles  # Store the last published angles
                break
        else:
            self.get_logger().warn("Timeout waiting for response from robot.")  # Timeout warning

    def extract_joint_angles(self, angle_line):
        angles = re.findall(r"[-+]?\d+", angle_line)
        angles_float = [float(angle) for angle in angles]
        if len(angles_float) != 16:  # Improved validation check
            self.get_logger().warn("Incorrect number of angles received.")
            return None
        return angles_float

    def publish_joint_angles(self, joint_angles):
        rad_angles = [math.radians(angle) for angle in joint_angles]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names  # Using dynamically loaded joint names
        joint_angles_rad = [rad_angles[0]] + rad_angles[-8:]
        self.get_logger().info(f"Publishing Joint Angles: {joint_angles_rad}")  # Using ROS2 logging for debugging
        msg.position = joint_angles_rad
        self.joint_state_publisher.publish(msg)
        self.get_logger().info('Publishing JointState in radians')

def main(args=None):
    rclpy.init(args=args)
    joint_angle_reader = JointAngleReader()

    try:
        rclpy.spin(joint_angle_reader)
    except KeyboardInterrupt:
        pass
    finally:
        joint_angle_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

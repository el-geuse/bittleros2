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
            self.serial_port = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.destroy_node()
            return
        self.timer = self.create_timer(0.05, self.query_and_publish_joint_angles)
        self.last_joint_angles = None

    def query_and_publish_joint_angles(self):
        # Send the 'j' command to query joint angles
        self.serial_port.write(b'j\n')

        # Wait briefly for the Bittle to respond
        #time.sleep(0.2)

        if self.serial_port.in_waiting:
            # Read and ignore the first line (joint indices)
            self.serial_port.readline()

            # The second line should contain the joint angles
            angle_line = self.serial_port.readline().decode().strip()
            joint_angles = self.extract_joint_angles(angle_line)

            if joint_angles is not None:
                self.last_joint_angles = joint_angles
                self.publish_joint_angles(joint_angles)
        
        if self.last_joint_angles is not None:
            self.publish_joint_angles(self.last_joint_angles)

    def extract_joint_angles(self, angle_line):
        # Use regular expressions to find all numbers in the string
        angles = re.findall(r"[-+]?\d+", angle_line)
        angles_float = [float(angle) for angle in angles]
    
        # Implement a check to see if these angles match a sequence of joint indices
        if angles_float == list(range(len(angles_float))):
            self.get_logger().warn("Extracted data matches joint indices sequence. Likely read the wrong line. :(")
            return None  # Indicates an error or wrong line was processed
        return angles_float

    def publish_joint_angles(self, joint_angles):
        if joint_angles is None:
            self.get_logger().error('Attempted to publish None joint_angles.')
            return

        rad_angles = [math.radians(angle) for angle in joint_angles]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['neck_joint', 'shrfs_joint', 'shrft_joint', 'shrrs_joint', 'shrrt_joint', 'shlfs_joint', 'shlft_joint', 'shlrs_joint', 'shlrt_joint']  # Adjust as necessary
        selected_angles = [rad_angles[0]] + rad_angles[-8:]
        msg.position = selected_angles
        self.get_logger().info(f"Publishing Joint Angles: {selected_angles}")
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

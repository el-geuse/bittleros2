#!/usr/bin/env python3
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import rclpy
import serial
import struct
import time
import re

dir_dict = {1: 'kwkF',           # walk forward
            -1: 'kbk',          # walk backwards
            2: 'kcrR',          # crawl to the right
            3: 'kcrL',          # crawl to the left
            4: 'ktrR',          # trot to the right
            5: 'ktrL',          # trot to the left
            0: 'kbalance',}      # balance (stay still)


class Driver(Node):

    def __init__(self, port='/dev/ttyS0'):
        super().__init__('cmd_vel_listener')
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.callback, 10)
        self.subscription  # prevent unused variable warning
        self.location_subscription = self.create_subscription(PoseStamped, "/person_location", self.location_callback, 10)

        self.dir = 0

        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1)

    def callback(self, msg):
        self.get_logger().info("Received a /cmd_vel message!")
        self.get_logger().info(
            "Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
        self.get_logger().info(
            "Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))

        if msg.linear.x > 0:
            dir = 1
        elif msg.linear.x < 0:
            dir = -1
        elif msg.angular.z > 0:
            dir = 5
        elif msg.angular.z < 0:
            dir = 4
        else:
            dir = 0

        if self.dir != dir:
            self.wrapper([dir_dict[dir], 0])
            self.dir = dir
    def location_callback(self, msg):
        # Example logic to move towards the person:
        # This is a simple approach; you'd likely want more sophisticated control logic.
        if msg.pose.position.x > 1.0:  # Assuming positive X is right
            self.send_teleop_command(2)  # Move right
            self.get_logger().info("Moving right towards the detected person.")
        elif msg.pose.position.x < -1.0:
            self.send_teleop_command(3)  # Move left
            self.get_logger().info("Moving left towards the detected person.")
        # Add more conditions as necessary for other directions

    def wrapper(self, task):  # Structure is [token, var=[], time]
        print(task)
        if len(task) == 2:
            self.serialWriteByte([task[0]])
        elif isinstance(task[1][0], int):
            self.serialWriteNumToByte(task[0], task[1])
        else:
            self.serialWriteByte(task[1])
        time.sleep(task[-1])

    def serialWriteNumToByte(self, token, var=[]):  # Only to be used for c m u b i l o
        # print("Num Token "); print(token);print(" var ");print(var);print("\n\n");
        if token == 'l' or token == 'i':
            var = list(map(lambda x: int(x), var))
            instrStr = token+struct.pack('b' * len(var), *var)+'~'

        elif token == 'c' or token == 'm' or token == 'u' or token == 'b':
            instrStr = token+str(var[0])+" "+str(var[1])+'\n'
        print("!!!!" + instrStr)
        self.ser.write(instrStr.encode())

    def serialWriteByte(self, var=[]):
        token = var[0][0]
        if (token == 'c' or token == 'm' or token == 'b' or token == 'u') and len(var) >= 2:
            instrStr = ""
            for element in var:
                instrStr = instrStr+element+" "
        elif token == 'l' or token == 'i':
            if(len(var[0]) > 1):
                var.insert(1, var[0][1:])
            var[1:] = list(map(lambda x: int(x), var[1:]))
            instrStr = token+struct.pack('b'*len(var[1:]), *var[1:])+'~'
        elif token == 'w' or token == 'k':
            instrStr = var[0] + '\n'
        else:
            instrStr = token
        print("!!!!!!! "+instrStr)
        self.ser.write(instrStr.encode())

def main(args=None):
    rclpy.init()

    driver = Driver()
    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
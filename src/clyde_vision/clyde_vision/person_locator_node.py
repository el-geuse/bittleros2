#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from clyde_vision.PersonLocator import LivePersonLocationDetector
import cv2

class PersonLocationNode(Node):
    def __init__(self):
        super().__init__('person_locator_node')
        self.subscriber = self.create_subscription(
            Image,
            '/image_raw/uncompressed',
            self.image_callback,
            10,
            )
        self.bridge = CvBridge()
        self.detector = LivePersonLocationDetector()

        self.modified_video_publisher = self.create_publisher(Image, '/modified_video', 10)

        self.add_on_set_parameters_callback(self.on_shutdown_callback)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        result = self.detector.process_frame(cv_image)
        if result:
            rect_width = 50  # Width of the rectangle
            rect_height = 100  # Height of the rectangle

            # Calculate the top-left corner of the rectangle
            top_left = (int(result["X"] - rect_width / 2), int(result["Y"] - rect_height / 2))

            # Calculate the bottom-right corner of the rectangle
            bottom_right = (int(result["X"] + rect_width / 2), int(result["Y"] + rect_height / 2))

            # Rectangle color (B, G, R) - Blue in this case
            color = (255, 0, 0)

            # Rectangle thickness
            thickness = 2

            # Draw the rectangle on the frame
            cv_image = cv2.rectangle(cv_image, top_left, bottom_right, color, thickness)
        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(cv_image, "rgb8")
        self.modified_video_publisher.publish(img_msg)


    def on_shutdown_callback(self):
        # Call the close method of your detector here
        self.get_logger().info('Shutting down, closing resources')
        self.detector.close()


def main(args=None):
    rclpy.init(args=['--remap _image_transport:=compressed'])
    person_locator_node = PersonLocationNode()
    rclpy.spin(person_locator_node)
    person_locator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

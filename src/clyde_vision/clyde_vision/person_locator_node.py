#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from clyde_vision.PersonLocator import LivePersonLocationDetector
from sensor_msgs.msg import CompressedImage

class PersonLocationNode(Node):
    def __init__(self):
        super().__init__('person_locator_node')
        self.subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10,
            )
        self.bridge = CvBridge()
        self.detector = LivePersonLocationDetector()

        self.add_on_set_parameters_callback(self.on_shutdown_callback)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        result = self.detector.process_frame(cv_image)
        if result:
            self.get_logger().info(f'Detected person at X: {result["X"]}, Z: {result["Z"]}')

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

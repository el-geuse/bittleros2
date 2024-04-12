#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from clyde_vision.FallDetector import FallDetector


class FallDetectorNode(Node):
    def __init__(self, model_path):
        super().__init__('fall_detector_node')
        self.subscriber = self.create_subscription(
            Image,
            '/image_raw/uncompressed',
            self.image_callback,
            10,
        )
        self.bridge = CvBridge()
        self.detector = FallDetector(model_path)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert fd image: %s' % str(e))
            return

        fall_probability = self.detector.fall_check(cv_image)

        fall_threshhold = 0.85
        if fall_probability > fall_threshhold:
            self.get_logger.info('Fall Detected: ', fall_probability, '%')
        else:
            self.get_logger.info('No Fall Detected: ', fall_probability, '%')


def main(args=None):

    rclpy.init(args=['--remap _image_transport:=compressed'])

    model_path = "/workspaces/clyderos2/Best_model_clyde_POV.h5"
    fall_detector_node = FallDetectorNode(model_path)

    rclpy.spin(fall_detector_node)

    fall_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

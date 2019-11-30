import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Byte, Bool, Float64, String # for mode reading and file input

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ModelLoader(Node):
    def __init__(self):
        super().__init__('model_loader')
        self.get_logger().info("Model loader starting...")

def main(args=None):
    rclpy.init(args=args)
    model_loader = ModelLoader()
    rclpy.spin(model_loader)
    model_loader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

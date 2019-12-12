import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Byte, Bool, Float64, String # for mode reading and file input
from sensor_msgs.msg import Joy

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import csv
import random #To randomly sort files into test and train folders (20% test, 80% train)

# each line in CSV file will include |time stamp|raw steering|raw speed|manual_max_speed|absolute_max_speed|

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.get_logger().info("Data collector starting...")

        self.declare_parameter("data_collection_rate", value=5.0) # Hz
        self.declare_parameter("absolute_max_speed", value=0.5)
        self.declare_parameter("triple_camera_mode", value=False)
        self.declare_parameter("center_camera_topic", value="/front_cam/color/image_raw")
        self.declare_parameter("right_camera_topic", value="")
        self.declare_parameter("left_camera_topic", value="")
        self.declare_parameter("data_store_root_dir", value="/media/data_usb/dataset")

        self.joystick_sub = self.create_subscription(Joy, 'joy', self.joy_cb, 2)
        self.raw_speed = 0
        self.raw_steering = 0

        self.collect_data_sub = self.create_subscription(Bool, "is_data_collecting", self.data_collecting_status_cb, 10)
        self.collect_data = False

        self.safety_sub = self.create_subscription(Bool, 'safety_on', self.safety_status_cb, 10)
        self.safety_on = True

        self.manual_max_speed_sub = self.create_subscription(Float64, 'manual_max_speed', self.manual_max_speed_cb, 10)
        self.manual_max_speed = 0

        self.collection_timer = self.create_timer(1.0/self.get_parameter("data_collection_rate").value, self.collect_data_cb)

    def data_collecting_status_cb(self, msg):
        last_state = self.collect_data
        self.collect_data = msg.data
        if last_state != self.collect_data:
            if self.collect_data:
                self.get_logger().info("Collecting Data")
            else:
                self.get_logger().info("Not Collecting Data")

    def safety_status_cb(self, msg):
        self.safety_on = msg.data

    def manual_max_speed_cb(self, msg):
        self.manual_max_speed = msg.data

    def joy_cb(self, msg):
        self.raw_speed = msg.axes[1]
        self.raw_steering = msg.axes[0]

    def collect_data_cb(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    rclpy.spin(data_collector)
    data_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

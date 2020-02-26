import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Byte, Bool, Float64, String # for mode reading and file input
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import csv, os, datetime, sys, time

class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')
        self.get_logger().info("Bridging topics")

        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.twist_pub = self.create_publisher(Twist, '/carla/ego_vehicle/twist_cmd', 1)
        self.camera_pub = self.create_publisher(Image, '/camera/color/image_raw', 1)
        # self.camera_info_pub = self.create_publisher(Twist, '/carla/ego_vehicle/twist_cmd', 1)

        # subscribe to latest steering and throttle commands and image topics to be relayed over
        self.create_subscription(AckermannDriveStamped, '/hw/cmd', self.cmd_cb, 1)
        self.create_subscription(Image, '/carla/ego_vehicle/camera/rgb/front/image_color', self.camera_cb, 1)
        # self.create_subscription(Image, '/carla/ego_vehicle/camera/rgb/front/camera_info', self.camera_info_cb, 1)


    def cmd_cb(self, msg):
        twist = Twist()
        twist.linear.x = 10*msg.drive.speed
        twist.angular.z = -1.57*msg.drive.steering_angle
        self.twist_pub.publish(twist)

    def camera_cb(self, msg):
        self.camera_pub.publish(msg)

    # def camera_info_cb(self, msg):
    #     pass

def main(args=None):
    rclpy.init(args=args)
    time.sleep(0.1) # magic sleep
    bridge = SimBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

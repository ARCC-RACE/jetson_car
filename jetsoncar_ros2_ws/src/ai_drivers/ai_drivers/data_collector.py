import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Byte, Bool, Float64, String # for mode reading and file input
from sensor_msgs.msg import Joy
from bno055.msg import Bno055

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Imu
import csv, os, datetime, sys, time
# each line in CSV file will include |time stamp|raw steering|raw speed|manual_max_speed|absolute_max_speed|IMU data

import xml.etree.ElementTree as ET
# the xml file will be at the daa collection root directory and contain all meta information for each dataset collected
"""
<data>
    <dataset1> # name of a dataset (each dataset should contain similar data for similar project function)
        <collection_period_1> # each collection period is each separate time segment data is addded to this dataset
            datetime
            location
            description
            notes
        </collection_period_1>
        <collection_period_2> # each collection period is each separate time segment data is addded to this dataset
            ...
        </collection_period_2>
    </dataset1>
    <dataset2> # name of a dataset (each dataset should contain similar data for similar project function)
        ...
    </dataset2>
</data>
"""

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.get_logger().info("Data collector starting...")

        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.declare_parameter("data_collection_rate", value=5.0) # Hz
        self.declare_parameter("absolute_max_speed", value=0.5)
        self.declare_parameter("ackermann_drive_topic", value="/hw/cmd")
        self.declare_parameter("triple_camera_mode", value=False)
        self.declare_parameter("center_camera_topic", value="/camera/color/image_raw")
        self.declare_parameter("center_camera_depth_topic", value="/camera/depth/image_rect_raw")
        # self.declare_parameter("right_camera_topic", value="")
        # self.declare_parameter("left_camera_topic", value="")
        self.declare_parameter("data_root_dir", value="/media/data_usb")
        self.declare_parameter("dataset_name", value="dataset")
        self.declare_parameter("dataset_description", value="This is where you can put information to describe your dataset")
        self.declare_parameter("entry_notes", value="If you would make a few notes about the dataset entry")

        self.field_names = ['time_stamp', 'raw_steering', 'raw_speed', 'max_speed', 'absolute_max_speed',
                                   'imu_calibration', 'quaternion_x', 'quaternion_y', 'quaternion_z', 'quaternion_w',
                                   'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z', 'linear_acceleration_x',
                                   'linear_acceleration_y', 'linear_acceleration_z']

        self.joystick_sub = self.create_subscription(Joy, 'joy', self.joy_cb, 2)
        self.raw_speed = 0
        self.raw_steering = 0

        self.collect_data_sub = self.create_subscription(Bool, "is_data_collecting", self.data_collecting_status_cb, 10)
        self.collect_data = False

        self.safety_sub = self.create_subscription(Bool, 'safety_on', self.safety_status_cb, 10)
        self.safety_on = True

        self.create_subscription(Float64, 'max_speed', self.max_speed_cb, 10)
        self.max_speed = 0

        self.collection_timer = self.create_timer(1.0/self.get_parameter("data_collection_rate").value, self.collect_data_cb)

        # subscribe to latest steering and throttle commands
        self.create_subscription(AckermannDriveStamped, self.get_parameter('ackermann_drive_topic').value, self.ackermann_cmd_cb, 1)
        self.last_ackermann_cmd = None
        self.imu_data = Imu() # last IMU message (not required data for recording)
        self.imu_status = Bno055() #last IMU message include calibration status

        # define subscribers for the image topics and variable to store the last image from each topic
        self.last_front_color_image = None
        self.last_front_depth_image = None

        self.create_subscription(Image, self.get_parameter('center_camera_topic').value, self.center_camera_cb, 1)
        self.create_subscription(Image, self.get_parameter('center_camera_depth_topic').value, self.center_camera_depth_cb, 1)


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

    def max_speed_cb(self, msg):
        self.max_speed = msg.data

    def joy_cb(self, msg):
        self.raw_speed = msg.axes[1]
        self.raw_steering = msg.axes[0]

    def ackermann_cmd_cb(self, data):
        self.last_ackermann_cmd = data

    def center_camera_cb(self, image):
        try:
            self.last_front_color_image = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            self.get_logger().error(e)

    def center_camera_depth_cb(self, image):
        try:
            self.last_front_depth_image = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            self.get_logger().error(e)

    def collect_data_cb(self):
        if self.collect_data and not (self.last_front_color_image is None and self.last_ackermann_cmd is None and self.last_front_depth_image is None):
            folder = self.get_parameter("dataset_name").value
            root_dir = self.get_parameter('data_root_dir').value
            dataset_dir = os.path.join(root_dir, folder)
            xml_manifest_path = os.path.join(root_dir, "manifest.xml")

            time_stamp = time.time()
            date = datetime.date.today()


            if not os.path.isdir(root_dir):
                self.get_logger().error("Data collection root dir does not exists: " + root_dir)
                sys.exit(1)

            if not os.path.exists(xml_manifest_path):
                tree = ET.ElementTree(ET.Element("datasets"))
                tree.write(xml_manifest_path) # this will make the xml file

            manifest_tree = ET.parse(xml_manifest_path)
            manifest = manifest_tree.getroot()
            datasets = manifest.findall('dataset')
            dataset_entry_exists = False
            current_dataset_index = None
            collection_entry_exists = False
            for i, dataset in enumerate(datasets):
                if dataset.attrib["name"] == folder:
                    dataset_entry_exists = True # do not add to dataset
                    current_dataset_index = i
                    for collection_entry in dataset.findall('collection_entry'): # collection entries created by date
                        if collection_entry.attrib["date"] == str(date):
                            collection_entry_exists = True

            if not dataset_entry_exists:
                dataset = ET.Element("dataset", {"name":folder, "description":self.get_parameter("dataset_description").value})
                current_dataset_index = len(manifest.findall('dataset'))
                manifest.append(dataset)

            if not collection_entry_exists:
                dataset =  manifest.findall('dataset')[current_dataset_index]
                collection_entry = ET.Element("collection_entry",  {"date":str(date), "notes":self.get_parameter("entry_notes").value,
                                                                    "rate":str(self.get_parameter("data_collection_rate").value),
                                                                    "triple_cam":str(self.get_parameter("triple_camera_mode").value),
                                                                    "start_stamp":str(time_stamp)})
                dataset.append(collection_entry)
                manifest_tree.write(xml_manifest_path)

            if not os.path.isdir(dataset_dir): # check if dataset already exists, else create it
                os.mkdir(dataset_dir)
                os.mkdir(os.path.join(dataset_dir, "color_images"))
                os.mkdir(os.path.join(dataset_dir, "depth_images"))
                with open(os.path.join(dataset_dir, "tags.csv"), 'w+') as csvfile:  # csv in test folder
                    # |time stamp|raw steering|raw speed|max_speed|absolute_max_speed|IMU data
                    csv_writer = csv.DictWriter(csvfile, self.field_names)
                    csv_writer.writeheader()

            # save all that data
            cv2.imwrite(os.path.join(dataset_dir, "color_images/" + str(time_stamp) + ".jpg"), self.last_front_color_image)  # write image into test folder
            cv2.imwrite(os.path.join(dataset_dir, "depth_images/" + str(time_stamp) + ".jpg"), self.last_front_depth_image)  # write image into test folder
            with open(os.path.join(dataset_dir, "tags.csv"), 'a') as csvfile: # append to csv file
                csv_writer = csv.writer(csvfile, self.field_names)
                new_data = [str(time_stamp), str(self.last_ackermann_cmd.drive.steering_angle), str(self.last_ackermann_cmd.drive.speed),
                            str(self.max_speed), str(self.get_parameter("absolute_max_speed").value), str(self.imu_status.sys_calibration),
                            str(self.imu_data.orientation.x), str(self.imu_data.orientation.y), str(self.imu_data.orientation.z),
                            str(self.imu_data.orientation.w), str(self.imu_data.angular_velocity.x), str(self.imu_data.angular_velocity.y),
                            str(self.imu_data.angular_velocity.z), str(self.imu_data.linear_acceleration.x), str(self.imu_data.linear_acceleration.y),
                            str(self.imu_data.linear_acceleration.z)]
                csv_writer.writerow(new_data)


def main(args=None):
    rclpy.init(args=args)
    time.sleep(0.1) # magic sleep
    data_collector = DataCollector()
    rclpy.spin(data_collector)
    data_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

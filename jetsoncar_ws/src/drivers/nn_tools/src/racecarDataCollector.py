#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import os
import getpass #gets username for filepaths
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import csv
import random #To randomly sort files into test and train folders (20% test, 80% train)

class RacecarDataCollector():

    def __init__(self, collection_fps=15, include_depth=False, include_imu=False, rgb_image_topic="/front_cam/color/image_raw",
                    depth_image_topic="/front_cam/aligned_depth_to_color/image_raw", imu_topic="/racecar/imu", imu_calibration_topic="/racecar/imu_info",
                    drive_control_topic="/racecar/muxed/ackermann_cmd", record_topic="/racecar/record_data", image_color_encoding="rgb8"):

        self.collection_fps = collection_fps
        self.include_depth = include_depth
        self.include_imu= include_imu

        self.isRecording = False #by default the system is not recording
        self.dir = os.path.join(os.path.join("/media/", getpass.getuser()), "/racecarDataset") #directory of expected USB flashdrive
        self.image_color_encoding = image_color_encoding

        self.lastRead = rospy.get_time() #get seconds in a float value

        #CSV header names
        self.feildNames = ['Time_stamp', 'Steering_angle', 'Speed', 'Imu_calibration', 'Quaternion_X', 'Quaternion_Y','Quaternion_Z','Quaternion_W',
            'Angular_velocity_X', 'Angular_velocity_Y', 'Angular_velocity_Z', 'Linear_acceleration_X', 'Linear_acceleration_Y', 'Linear_acceleration_Z' ]

        #create dataset directory
        #if directory is alread made do nothing
        #looking for usb flash drive with name racecarDataset
        if os.path.exists(self.dir):
            try:
                os.makedirs(os.path.join(self.dir, "/dataset/training_set"))
                os.makedirs(os.path.join(self.dir, "/dataset/test_set"))
                rospy.loginfo("Dataset directory created")
                #make the csv files
                #prepare CSV files by writing header (starts with blank csv)
                with open(os.path.join(self.dir, "/dataset/training_set/tags.csv"), 'w') as csvfile: #csv in train folder
                    csv_writer = csv.DictWriter(csvfile, self.feildNames)
                    csv_writer.writeheader()
                with open(os.path.join(self.dir, "/dataset/test_set/tags.csv"), 'w') as csvfile: #csv in test folder
                    csv_writer = csv.DictWriter(csvfile, self.feildNames)
                    csv_writer.writeheader()
            except OSError:
                #if folders exists it is expected that the csv files exist as well
                rospy.loginfo("Dataset directories already exists")
        else:
            rospy.logerr("No racecarDataset flash drive detected")
            exit(1)

        rospy.Subscriber(record_topic, Bool, self.updateRecordStatus) #status on whether to record data or not

        rospy.loginfo("Waiting for ackermann_cmd to be published before starting recording...")
        lastAckermann = rospy.wait_for_message(drive_control_topic, AckermannDriveStamped)
        rospy.loginfo("Recording started")

        rospy.Subscriber(drive_control_topic, AckermannDriveStamped, self.newDriveData) #drive control data from user input
        rospy.Subscriber(rgb_image_topic, Image, self.newImage) #video
        rospy.Subscriber(depth_image_topic, Image, self.newDepth) #depth
        rospy.Subscriber(imu_topic, Imu, self.newIMUData) #IMU
        rospy.Subscriber(imu_calibration_topic, Imu, self.newIMUCalibData) #IMU info/calibration

    #record data with time stamp
    def newImage(self, new_image):
        timeStamp = rospy.get_time()

        if(isRecording and timeStamp - self.lastRead > 1.0/collection_fps): # fps data recording (delay = 1/desired fps)

            try:
                bridge = CvBridge() #convert ros image to cv compatible image
                image = bridge.imgmsg_to_cv2(new_image, desired_encoding=self.image_color_encoding)  #throws out non-rgb8 images
                depth = bridge.imgmsg_to_cv2(self.depth_data, desired_encoding="passthrough")
                print(depth.shape)
            except CvBridgeError as e:
                rospy.logdebug(e)
                return
            self.lastRead = rospy.get_time() #Update lastRead time stamp

            #create a new row to add into a csv file
            if self.include_imu:
                new_data = [str(timeStamp), str(self.lastAckermann.drive.steering_angle), str(self.lastAckermann.drive.speed), str(self.imu_calibdata.sysCalibration), str(self.imu_data.orientation.x),
                    str(self.imu_data.orientation.y), str(self.imu_data.orientation.z), str(self.imu_data.orientation.w), str(self.imu_data.angular_velocity.x), str(self.imu_data.angular_velocity.y),
                    str(self.imu_data.angular_velocity.z), str(self.imu_data.linear_acceleration.x), str(self.imu_data.linear_acceleration.y), str(self.imu_data.linear_acceleration.z)]
            else:
                new_data = [str(timeStamp), str(self.lastAckermann.drive.steering_angle), str(self.lastAckermann.drive.speed)]
            #determine whether data goes into train or test folders
            fileSelector = random.randint(0, 10) #random number between 0 and 9 inclusive
            #put new_data into csv
            if(fileSelector < 8): #send data to train folder
                cv2.imwrite(os.path.join(os.path.join(self.dir, "/dataset/training_set/"), str(timeStamp) + ".jpg", image); #write image into train folder
                if self.include_depth:
                    cv2.imwrite(os.path.join(os.path.join(self.dir, "/dataset/training_set/"), str(timeStamp) + "_depth.jpg"), depth); #write depth image into train folder
                #update csv file
                with open(os.path.join(self.dir, "/dataset/training_set/tags.csv"), 'a') as csvfile: # append to csv file
                    csv_writer = csv.writer(csvfile, self.feildNames)
                    csv_writer.writerow(new_data)

            else: #send data to test folder
                cv2.imwrite(os.path.join(os.path.join(self.dir, "/dataset/test_set/"), str(timeStamp) + ".jpg"), image); #write image into test folder
                if self.include_depth:
                    cv2.imwrite(os.path.join(os.path.join(self.dir, "/dataset/test_set/"), str(timeStamp) + "_depth.jpg"), depth); #write depth image into test folder
                #update csv file
                with open(os.path.join(self.dir, "/dataset/test_set/tags.csv"), 'a') as csvfile: # append to csv file
                    csv_writer = csv.writer(csvfile, self.feildNames)
                    csv_writer.writerow(new_data)

            rospy.logdebug("Data recorded") #Everytime data is added to test/training folders


    #udpate steering and speed values
    def newDriveData(self, data):
        self.lastAckermann = data

    #update record update record status
    def updateRecordStatus(self, data):
        self.isRecording = data.data

    #get IMU data for recording
    def newIMUData(self, data):
        self.imu_data = data

    #get IMU Calibration data for recording
    def newIMUCalibData(self, data):
        self.imu_calib_data = data

    #get stereoscopic depth data for recording
    def newDepth(self, image):
        self.depth_data = image

if __name__=="main":
    #program starts running here
    rospy.init_node("data_collector")
    include_imu = rospy.get_param('~include_imu', False)
    include_depth = rospy.get_param('~include_depth', False)
    collection_fps = rospy.get_param('~collection_fps', 15)
    image_color_encoding = rospy.get_param('~image_color_encoding', "bgr8")

    data_collector = RacecarDataCollector(collection_fps, include_depth, include_imu, image_color_encoding=image_color_encoding)
    rospy.spin() #keep node running while callbacks are handled

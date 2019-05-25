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
import csv
import random #To randomly sort files into test and train folders (20% test, 80% train)

isRecording = False #by default the system is not recording
dir = "/media/" + getpass.getuser() + "/racecarDataset" #directory of expected USB flashdrive

#record data with time stamp
def newImage(new_image):
    timeStamp = rospy.get_time()

    global isRecording
    global lastRead
    global dir

    if(isRecording and timeStamp - lastRead > 0.0666666667): # ~15 fps data recording (1/desired fps)

        try:
            bridge = CvBridge() #convert ros image to cv compatible image
            image = bridge.imgmsg_to_cv2(new_image, desired_encoding="bgr8")  #throws out non-rgb8 images
        except CvBridgeError as e:
            rospy.logdebug(e)
            return
        lastRead = rospy.get_time() #Update lastRead time stamp

        fileSelector = random.randint(0, 10) #random number between 0 and 9 inclusive
        if(fileSelector < 8): #send data to train folder
            cv2.imwrite(dir + "/dataset/training_set/" + str(timeStamp) + ".jpg", image); #write image into train folder
            #update csv file
            global lastAckermann #Only the current acting (last updated) ackermann message is important
            with open(dir + "/dataset/training_set/tags.csv", 'a') as csvfile: # append to csv file
                feildNames = ['Time_stamp', 'Steering_angle', 'Speed']
                csv_writer = csv.writer(csvfile, feildNames)
                newData = [str(timeStamp), str(lastAckermann.drive.steering_angle), str(lastAckermann.drive.speed)]
                csv_writer.writerow(newData)

        else: #send data to test folder
            cv2.imwrite(dir + "/dataset/test_set/" + str(timeStamp) + ".jpg", image); #write image into test folder
            #update csv file
            global lastAckermann #Only the current acting (last updated) ackermann message is important
            with open(dir + "/dataset/test_set/tags.csv", 'a') as csvfile: # append to csv file
                feildNames = ['Time_stamp', 'Steering_angle', 'Speed']
                csv_writer = csv.writer(csvfile, feildNames)
                newData = [str(timeStamp), str(lastAckermann.drive.steering_angle), str(lastAckermann.drive.speed)]
                csv_writer.writerow(newData)

        rospy.logdebug("Data recorded") #Everytime data is added to test/training folders


#udpate steering and speed values
def newDriveData(data):
    global lastAckermann
    lastAckermann = data

#update record update record status
def updateRecordStatus(data):
    global isRecording
    isRecording = data.data

#program starts running here
rospy.init_node("data_collector")
lastRead = rospy.get_time() #get seconds in a float value

#create dataset directory
#if directory is alread made do nothing
#looking for usb flash drive with name racecarDataset
if os.path.exists(dir):
    try:
        os.makedirs(dir + "/dataset/training_set")
        os.makedirs(dir + "/dataset/test_set")
        rospy.loginfo("Dataset directory created")
        #make the csv files
        #prepare CSV files by writing header (starts with blank csv)
        with open(dir + "/dataset/training_set/tags.csv", 'w') as csvfile: #csv in train folder
            feildNames = ['Time_stamp', 'Steering_angle', 'Speed']
            csv_writer = csv.DictWriter(csvfile, feildNames)
            csv_writer.writeheader()
        with open(dir + "/dataset/test_set/tags.csv", 'w') as csvfile: #csv in test folder
            feildNames = ['Time_stamp', 'Steering_angle', 'Speed']
            csv_writer = csv.DictWriter(csvfile, feildNames)
            csv_writer.writeheader()
    except OSError:
        #if folders exists it is expected that the csv files exist as well
        rospy.loginfo("Dataset directories already exists")
else:
    rospy.logerr("No racecarDataset flash drive detected")
    exit(1)

rospy.Subscriber("/racecar/record_data", Bool, updateRecordStatus) #status on whether to record data or not

lastAckermann = rospy.wait_for_message("/racecar/muxed/ackermann_cmd", AckermannDriveStamped)

rospy.Subscriber("/racecar/muxed/ackermann_cmd", AckermannDriveStamped, newDriveData) #drive control data from user input
rospy.Subscriber("/front_cam/color/image_raw", Image, newImage) #video
rospy.spin() #keep node running while callbacks are handled

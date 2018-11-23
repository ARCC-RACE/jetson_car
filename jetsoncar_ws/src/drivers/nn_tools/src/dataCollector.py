#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
import csv
import random #To randomly sort files into test and train folders (20% test, 80% train)

#record data with time stamp
def newImage(new_image):
    timeStamp = rospy.get_time()

    global lastRead
    if(timeStamp - lastRead > 0.0666666667): # ~15 fps data recording (1/desired fps)

        try:
            bridge = CvBridge() #convert ros image to cv compatible image
            image = bridge.imgmsg_to_cv2(new_image, desired_encoding="rgb8")  #throws out non-rgb8 images
        except CvBridgeError as e:
            rospy.logdebug(e)
            return
        lastRead = rospy.get_time() #Update lastRead time stamp

        fileSelector = random.randint(0, 10) #random number between 0 and 9 inclusive
        if(fileSelector < 8): #send data to train folder
            cv2.imwrite( "../dataset/training_set/" + str(timeStamp) + ".jpg", image); #write image into train folder
            #update csv file
            global lastAckermann #Only the current acting (last updated) ackermann message is important
            with open("../dataset/training_set/tags.csv", 'a') as csvfile: # append to csv file
                feildNames = ['Time_stamp', 'Steering_angle', 'Speed']
                csv_writer = csv.writer(csvfile, feildNames)
                newData = [str(timeStamp), str(lastAckermann.drive.steering_angle), str(lastAckermann.drive.speed)]
                csv_writer.writerow(newData)

        else: #send data to test folder
            cv2.imwrite( "../dataset/test_set/" + str(timeStamp) + ".jpg", image); #write image into test folder
            #update csv file
            global lastAckermann #Only the current acting (last updated) ackermann message is important
            with open("../dataset/test_set/tags.csv", 'a') as csvfile: # append to csv file
                feildNames = ['Time_stamp', 'Steering_angle', 'Speed']
                csv_writer = csv.writer(csvfile, feildNames)
                newData = [str(timeStamp), str(lastAckermann.drive.steering_angle), str(lastAckermann.drive.speed)]
                csv_writer.writerow(newData)

        rospy.loginfo("Data recorded") #Everytime data is added to test/training folders


#udpate steering and speed values
def newDriveData(data):
    global lastAckermann
    lastAckermann = data


#prepare CSV files by writing header (starts with blank csv)
with open("../dataset/training_set/tags.csv", 'w') as csvfile: #csv in train folder
    feildNames = ['Time_stamp', 'Steering_angle', 'Speed']
    csv_writer = csv.DictWriter(csvfile, feildNames)
    csv_writer.writeheader()
with open("../dataset/test_set/tags.csv", 'w') as csvfile: #csv in test folder
    feildNames = ['Time_stamp', 'Steering_angle', 'Speed']
    csv_writer = csv.DictWriter(csvfile, feildNames)
    csv_writer.writeheader()

#program starts running here
rospy.init_node("data_collector")
lastRead = rospy.get_time() #get seconds in a float value

lastAckermann = rospy.wait_for_message("/racecar/ackermann_cmd", AckermannDriveStamped)

rospy.Subscriber("/racecar/ackermann_cmd", AckermannDriveStamped, newDriveData) #drive control data from user input
rospy.Subscriber("/front_cam/depth/image_raw", Image, newImage) #video
rospy.spin() #keep node running while callbacks are handled

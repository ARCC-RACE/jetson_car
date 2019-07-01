#!/usr/bin/env python

import rospy
import utils # for image preprocessing
import numpy as np
import os
import cv2
from rosey import Rosey

# Keras imports to create a convolutional neural network using tensorflow on the low level
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Lambda, Dropout
from keras.models import load_model
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

# callback that runs when a new image is recieved from realsense camera
def newImage(new_image):
    timeStamp = rospy.get_time()

    global lastRead
    global steering_prediction
    global rosey #CNN model
    if(timeStamp - lastRead > 0.033333333): # ~30 fps desired data flow (1/desired fps)

        try:
            bridge = CvBridge() # convert ros image to cv compatible image
            image = bridge.imgmsg_to_cv2(new_image, desired_encoding="rgb8")  #throws out non-rgb8 images
        except CvBridgeError as e:
            rospy.logdebug(e)
            return
        lastRead = rospy.get_time() # update lastRead time stamp

#        startProcessing = rospy.get_time() #DEBUGGING
#        cv2.imshow("image", image)         #DEBUGGING
#        cv2.waitKey(0)                     #DEBUGGING
        image = utils.preprocess(image) # preprocess image (crop, resize, rgb2yuv)
#        cv2.imshow("image", image)         #DEBUGGING
#        cv2.waitKey(0)                     #DEBUGGING

        image = np.array([image]) # give the model a 4D array
        steering_prediction = float(rosey.predict(image, batch_size=1))
#        processTime = rospy.get_time() - startProcessing #DEBUGGING
#        print(processTime) # prints time to process image and make prediction in seconds
        rospy.logdebug("Steering prediction from Rosey: %f", steering_prediction)

def newVel(data):
    global drive_speed
    global use_cruise_velocity
    if(use_cruise_velocity):
        drive_speed = data.data

dir_path = os.path.dirname(os.path.realpath(__file__)) # returns filepath to the location of the python file
#rosey = load_model(dir_path + '/../models/rosey.h5')
roseyObj = Rosey()
roseyObj.build_model()
rosey = roseyObj.model
rosey.load_weights(dir_path + '/../models/rosey.h5')
rosey._make_predict_function() # build and compile the function on the GPU (before threading)

rospy.init_node('rosey')

drive_speed = rospy.get_param('~speed', 2)
use_cruise_velocity = rospy.get_param('~use_cruise_velocity', False)

lastRead = rospy.get_time()
steering_prediction = 0

pub = rospy.Publisher("/racecar/autonomous/ackermann_cmd", AckermannDriveStamped, queue_size=1) # drive control publisher
rospy.Subscriber("/front_cam/color/image_raw", Image, newImage)  # input video
rospy.Subscriber("/racecar/cruise_velocity", Float64, newVel)  # input video

rate = rospy.Rate(50) #50Hz publshing rate

while not rospy.is_shutdown():
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle = steering_prediction
    drive_msg.drive.speed = drive_speed # constant speed for now, safety controlled in mux node
    drive_msg.header.stamp = rospy.Time.now()
    pub.publish(drive_msg)
    rate.sleep()

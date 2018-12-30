#!/usr/bin/env python 

import getpass
import numpy 
import cv2

dir = "/media/" + getpass.getuser() + "/racecarDataset/dataset" #directory of expected USB flashdrive
img = cv2.imread(dir+"/test_set/1543390907.89.jpg",1) #load in colored image

cv2.imshow('image',img)
cv2.waitKey(0)

img = img[220:-1, :, :] # remove the sky
cv2.imshow('image',img)
cv2.waitKey(0)

img = cv2.resize(img, (200, 66), cv2.INTER_AREA)
cv2.imshow('image',img)
cv2.waitKey(0)

img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
cv2.imshow('image',img)
cv2.waitKey(0)

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import csv

# Keras imports to create a convolutional neural network using tensorflow on the low level
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Lambda, Dropout
#to save the model periodically as checkpoints for loading later
from keras.callbacks import ModelCheckpoint
#popular optimization strategy that uses gradient descent
from keras.optimizers import Adam
#helper class to define input shape and generate training images given image paths & steering angles
from utils import INPUT_SHAPE, batch_generator

#################################################################
model = Sequential()  # linear stack of layers
# normalize the image  to avoid saturation and make the gradients work better
model.add(Lambda(lambda x: x / 127.5 - 1.0, input_shape=INPUT_SHAPE))  # 127.5-1.0 = experimental value from udacity self driving car course
# 24 5x5 convolution kernels with 2x2 stride and activation function Exponential Linear Unit (to avoid vanishing gradient problem)
model.add(Conv2D(24, 5, activation="elu", strides=2))
model.add(Conv2D(36, 5, activation="elu", strides=2))
model.add(Conv2D(48, 5, activation="elu", strides=2))
model.add(Conv2D(64, 3, activation="elu"))  # stride = 1x1
model.add(Conv2D(64, 3, activation="elu"))  # stride = 1x1

model.add(Dropout(0.5))  # magic number from udacity self driving car course
# turn convolutional feature maps into a fully connected ANN
model.add(Flatten())
model.add(Dense(100, activation="elu"))
model.add(Dense(50, activation="elu"))
model.add(Dense(10, activation="elu"))
model.add(Dense(1))  # No need for activation function because this is the output and it is not a probability
model.summary()  # print a summary representation of model
model.compile(loss='mean_squared_error', optimizer=Adam(1.0e-4))
#################################################################

model.load_weights('model.h5', by_name=True)
model.save("rosey_python3.h5")

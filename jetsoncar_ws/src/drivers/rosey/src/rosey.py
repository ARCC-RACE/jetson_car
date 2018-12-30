#!/usr/bin/env python

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

class Rosey:

    def __init__(self, data_directory = "../data", batch_size=40, validation_steps=1000, nb_epochs=10, steps_per_epoch=1500):
        self.data_dir = data_directory
        self.batch_size = batch_size
        self.validation_steps = validation_steps
        self.nb_epochs = nb_epochs
        self.steps_per_epoch = steps_per_epoch

    def build_model(self):
        print("Building model...")

        #based off of Nvidia's Dave 2 system
        #raw image height = 480, width = 640
        #NN input image shape (crop and resize raw) = 66x200x3: INPUT_SHAPE = (IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS)

        self.model = Sequential() #linear stack of layers
        #normalize the image  to avoid saturation and make the gradients work better
        self.model.add(Lambda(lambda x: x/127.5-1.0, input_shape=INPUT_SHAPE)) #127.5-1.0 = experimental value from udacity self driving car course
        #24 5x5 convolution kernels with 2x2 stride and activation function Exponential Linear Unit (to avoid vanishing gradient problem)
        self.model.add(Conv2D(24, 5, activation="elu", strides=2))
        self.model.add(Conv2D(36, 5, activation="elu", strides=2))
        self.model.add(Conv2D(48, 5, activation="elu", strides=2))
        self.model.add(Conv2D(64, 3, activation="elu")) #stride = 1x1
        self.model.add(Conv2D(64, 3, activation="elu")) #stride = 1x1

        self.model.add(Dropout(0.5)) #magic number from udacity self driving car course
        #turn convolutional feature maps into a fully connected ANN
        self.model.add(Flatten())
        self.model.add(Dense(100, activation="elu"))
        self.model.add(Dense(50, activation="elu"))
        self.model.add(Dense(10, activation="elu"))
        self.model.add(Dense(1)) #No need for activation function because this is the output and it is not a probability
        self.model.summary() #print a summary representation of model



    def train_model(self):
        #filepath for save = rosey.epoch-loss.h5 (rosey-{epoch:03d}.h5 is another option)
        #saves epoch with the minimum val_loss
        checkpoint = ModelCheckpoint('rosey.{epoch:03d}-{val_loss:.2f}.h5', # filepath = working directory/
            monitor='val_loss',
            verbose=0,
            save_best_only=True,
            mode='auto')

        #compile model with stochastic gradient descent
        self.model.compile(loss='mean_squared_error', optimizer=Adam(1.0e-4)) #learning rate of 1.0e-4 udacity= magic number from udacity

        #batch_generator(data_dir, image_paths, steering_angles, batch_size, is_training):
        #generator, steps_per_epoch=None, epochs=1, verbose=1, callbacks=None, validation_data=None, validation_steps=None,
        # class_weight=None, max_queue_size=10, workers=1, use_multiprocessing=False, shuffle=True, initial_epoch=0
        self.model.fit_generator(batch_generator(self.data_dir + "/training_set" , self.X_training, self.Y_training, self.batch_size, True),
            self.steps_per_epoch, self.nb_epochs, max_queue_size=1,
            validation_data=batch_generator(self.data_dir + "/test_set", self.X_test, self.Y_test, self.batch_size, False),
            #validation_steps=len(self.X_test), #Takes wwwwaaayyyyyy too long
            validation_steps=self.validation_steps,
            callbacks=[checkpoint],
            verbose=1)


    def load_data(self):
        #load training set
        print("Loading training set...")
        with open(self.data_dir + "/training_set/tags.csv") as csvfile:
            reader = csv.DictReader(csvfile)
            self.X_training = ()
            self.Y_training = ()
            for row in reader:
                #print(row['Time_stamp'] + ".jpg", row['Steering_angle'])
                self.X_training += (row['Time_stamp'] + ".jpg",) #get image path
                self.Y_training += (float(row['Steering_angle']),)

        #load test set
        print("Loading test set...")
        with open(self.data_dir + "/test_set/tags.csv") as csvfile:
            reader = csv.DictReader(csvfile)
            self.X_test = ()
            self.Y_test = ()
            for row in reader:
                #print(row['Time_stamp'] + ".jpg", row['Steering_angle'])
                self.X_test += (row['Time_stamp'] + ".jpg",) #get image path
                self.Y_test += (float(row['Steering_angle']),)

if __name__ == "__main__":
    rosey = Rosey(data_directory = "../../nn_tools/dataset",
        batch_size=40, validation_steps=2500, nb_epochs=10,
        steps_per_epoch=10000)
    rosey.load_data()
    rosey.build_model()
    rosey.train_model()

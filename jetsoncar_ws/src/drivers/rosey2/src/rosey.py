#!/usr/bin/env python

import numpy as np
import cv2
import csv
import time
import os
import getpass #gets username for filepaths
import platform #determines wether OS in windows or ubuntu

# Keras imports to create a convolutional neural network using tensorflow on the low level
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Lambda, Dropout
#to save the model periodically as checkpoints for loading later
from keras.callbacks import ModelCheckpoint
#popular optimization strategy that uses gradient descent
from keras.optimizers import Adam
#add regularizer to limit overfitting
from keras import regularizers
#Gotta have them graphs
from keras.callbacks import TensorBoard
#helper class to define input shape and generate training images given image paths & steering angles
from utils import INPUT_SHAPE, batch_generator

class Rosey:

    def __init__(self, data_directory = "../data", datasets="dataset", batch_size=40, validation_steps=1000, nb_epochs=10, steps_per_epoch=1500, regularizer=0.01):
        self.data_dir = data_directory
        self.datasets = datasets
        self.batch_size = batch_size
        self.validation_steps = validation_steps
        self.nb_epochs = nb_epochs
        self.steps_per_epoch = steps_per_epoch
        self.regularizer = regularizer

    def build_model(self):
        print("Building model...")

        #based off of Nvidia's Dave 2 system
        #raw image height = 480, width = 640
        #NN input image shape (crop and resize raw) = 66x200x3: INPUT_SHAPE = (IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS)

        self.model = Sequential() #linear stack of layers
        #normalize the image  to avoid saturation and make the gradients work better
        self.model.add(Lambda(lambda x: x/127.5-1.0, input_shape=INPUT_SHAPE)) #127.5-1.0 = experimental value from udacity self driving car course
        #24 5x5 convolution kernels with 2x2 stride and activation function Exponential Linear Unit (to avoid vanishing gradient problem)
        self.model.add(Conv2D(24, 5, activation="elu", strides=2, kernel_initializer='he_normal', kernel_regularizer=regularizers.l1(self.regularizer)))
        self.model.add(Conv2D(36, 5, activation="elu", strides=2, kernel_initializer='he_normal', kernel_regularizer=regularizers.l1(self.regularizer)))
        self.model.add(Conv2D(48, 5, activation="elu", strides=2, kernel_initializer='he_normal', kernel_regularizer=regularizers.l1(self.regularizer)))
        self.model.add(Conv2D(64, 3, activation="elu")) #stride = 1x1
        self.model.add(Conv2D(64, 3, activation="elu")) #stride = 1x1

        self.model.add(Dropout(0.5)) #magic number from udacity self driving car course
        #turn convolutional feature maps into a fully connected ANN
        self.model.add(Flatten())
        self.model.add(Dense(100, activation="elu", kernel_initializer='he_normal', kernel_regularizer=regularizers.l1(self.regularizer)))
        self.model.add(Dense(50, activation="elu", kernel_initializer='he_normal', kernel_regularizer=regularizers.l1(self.regularizer)))
        self.model.add(Dense(10, activation="elu", kernel_initializer='he_normal', kernel_regularizer=regularizers.l1(self.regularizer)))
        self.model.add(Dense(1)) #No need for activation function because this is the output and it is not a probability

        #Setup tensorboard for viewing model development
        timeStamp = time.time()
        path = os.path.dirname(os.path.realpath(__file__)) #get python file path
        self.tensorboard = TensorBoard(log_dir="{}/logs/{}".format(path, timeStamp))
        print("Run `tensorboard --logdir=\"{}/logs/{}\"` and see `http://localhost:6006` to see CNN status".format(path, timeStamp) + "\n\n")

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
        self.model.fit_generator(batch_generator(self.data_dir, self.datasets, self.X_training, self.Y_training, self.batch_size, True),
            self.steps_per_epoch, self.nb_epochs, max_queue_size=1,
            validation_data=batch_generator(self.data_dir, self.datasets, self.X_test, self.Y_test, self.batch_size, False),
            #validation_steps=len(self.X_test), #Takes wwwwaaayyyyyy too long
            validation_steps=self.validation_steps,
            callbacks=[checkpoint, self.tensorboard],
            verbose=1)

    def train_model_from_npy(self, save_dataset=True):
        checkpoint = ModelCheckpoint('rosey.{epoch:03d}-{val_loss:.2f}.h5', # filepath = working directory/
            monitor='val_loss',
            verbose=0,
            save_best_only=True,
            mode='auto')
        #compile model with stochastic gradient descent
        self.model.compile(loss='mean_squared_error', optimizer=Adam(1.0e-4)) #learning rate of 1.0e-4 udacity= magic number from udacity
        #build the input(x) output(y) arrays
        print("\nBuilding FAT numpy array of augmented datatset... (this may take a while)")
        x_images, y_steers = fat_npy_builder(self.data_dir, self.datasets, self.X_training, self.Y_training, self.X_test, self.Y_test, self.temporal_size, total_size=25000)

        #option to save the generated numpy so it can be reused later by train_model_from_old_npy()
        if save_dataset:
            print("Saving dataset npy...")
            np.save(os.path.join(self.data_dir, 'x_images.npy'), x_images)
            np.save(os.path.join(self.data_dir, 'y_steers.npy'), y_steers)
            print("Dataset saved!")

        print("Finished building, beginning training of neural network")
        self.model.fit(x_images, y_steers, self.batch_size, nb_epoch=50, verbose=1, validation_split=0.2, shuffle=True, callbacks=[checkpoint, self.tensorboard])

    #lets you load in an old numpy file that contains the augmented dataset generated in train_model_from_npy()
    def train_model_from_old_npy(self):
        checkpoint = ModelCheckpoint('rosey.{epoch:03d}-{val_loss:.2f}.h5', # filepath = working directory/
            monitor='val_loss',
            verbose=0,
            save_best_only=True,
            mode='auto')
        #compile model with stochastic gradient descent
        self.model.compile(loss='mean_squared_error', optimizer=Adam(1.0e-4)) #learning rate of 1.0e-4 udacity= magic number from udacity
        #build the input(x) output(y) arrays
        print("\nLoading FAT numpy array of augmented datatset... (this may take a while)")

        #option to save the generated numpy so it can be reused later
        x_images = np.load(os.path.join(self.data_dir, 'x_images.npy'))
        y_steers = np.load(os.path.join(self.data_dir, 'y_steers.npy'))

        print("Finished loading, beginning training of neural network")
        self.model.fit(x_images, y_steers, self.batch_size, nb_epoch=50, verbose=1, validation_split=0.2, shuffle=True, callbacks=[checkpoint, self.tensorboard])

    def load_data(self):
        #load training set
        print("Loading training set...")
        #these will be 2D arrays where each row represents a dataset
        self.X_training = []
        self.Y_training = []
        for dataset in self.datasets:
            X_training = ()
            Y_training = ()
            with open(os.path.join(os.path.join(self.data_dir, dataset), "training_set/tags.csv")) as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    #print(row['Time_stamp'] + ".jpg", row['Steering_angle'])
                    X_training += (row['Time_stamp'] + ".jpg",) #get image path
                    Y_training += (float(row['Steering_angle']),)
            self.X_training.append(X_training)
            self.Y_training.append(Y_training)

        #load test set
        print("Loading test set...")
        #these will be 2D arrays where each row represents a dataset
        self.X_test = []
        self.Y_test = []
        for dataset in self.datasets:
            X_test = ()
            Y_test = ()
            with open(os.path.join(os.path.join(self.data_dir, dataset), "test_set/tags.csv")) as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    #print(row['Time_stamp'] + ".jpg", row['Steering_angle'])
                    X_test += (row['Time_stamp'] + ".jpg",) #get image path
                    Y_test += (float(row['Steering_angle']),)
            self.X_test.append(X_test)
            self.Y_test.append(Y_test)

if __name__ == "__main__":
    #determine which file system to use for reading racecarDataset. It may not always be optimal to read from USB drive
    dir = input("Enter root dataset directory (i.e. ~/racecarDatasets). If left blank the USB root dataset directory will be used.")
    if dir.strip() == "":
        if(platform.system() == 'Windows'):
            dir = "E:\\" #directory of expected USB flashdrive on Windows
            print("Windows detected! Searching " + dir + " for dataset")
        else:
            dir = "/media/" + getpass.getuser() + "/racecarDataset" #directory of expected USB flashdrive on Linux
            print("Linux detected! Searching " + dir + " for dataset")

    print("Dataset Location: " + dir + "\n")

    #This function lets you combine ex. dataset with dataset1 and dataset2 for a single trainig session. This makes adding in more data easy and seperates data collection periods
    special_dataset_read = input("Would you like to combine multiple datasets in the root dataset directory for training? (y/n)")
    if special_dataset_read.lower().strip() == 'y':
        print("The dataset root should be configured to cotnain files in the format " + os.path.join(dir + "dataset") + ",1,2,3,4,...")
        datasets = input("Enter in the sufixes for the datasets to read from: (ex.  `,3,5`  will read from "  + os.path.join(dir + "dataset") + ", " +  os.path.join(dir + "dataset") + "3, and " + os.path.join(dir + "dataset") +"5)").split(',')
        for i,suffix in enumerate(datasets):
            datasets[i] = "dataset"+suffix
    else:
        datasets = ["dataset"]
        print("Using default " + os.path.join(dir + "dataset") + " for data")

    print("Datasets to read from: " + str(datasets) + "\n\n")

    rosey = Rosey(data_directory=dir, datasets=datasets,
        batch_size=40, validation_steps=1000, nb_epochs=20,
        steps_per_epoch=1000, regularizer=0)

    rosey.load_data()
    rosey.build_model()
    #rosey.train_model()
    #rosey.train_model_from_npy()
    rosey.train_model_from_old_npy()

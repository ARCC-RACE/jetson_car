#!/usr/bin/env python

import hal_env

import gym
import gym_gazebo
import time
import numpy
import random
import time
import matplotlib
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from gym import wrappers
from liveplot import LivePlot

#utils from Dave2 net Rosey was modeled after
import utils

from memory import Memory

from keras import Sequential, optimizers
from keras.layers import Dense, Flatten, Conv2D
from keras.models import load_model

"""
init replay memory
init action-value function Q with ranfom weights

repeat
    select action a, with probability e select one at random otherwise do a = argmax'Q(s,a')
    carry out action a
    observe reward and new state s'
    store experience <s, a, r, s'> into replay memory

    sample random transitions <ss, aa, rr, ss'> from replay memory
    calculate target for each miniBatch transition
        if ss' is terminal state then tt = rr
        otherwise tt = rr + gamma*maxa'Q(ss',aa')
    train the Q network using (tt - Q(ss, aa))^2 as loss function

    s = s'
until terminated

"""

"""

Convolutional Neural Network based off DeepMind Architecture
Convert image to YUV encoding (see Rosey)
W = (W-F+2P)/S+1 -> calc convolutional layer output

       input       kernel  stride  num_filters    activation   output
conv1  212x60x12   8x8     4       32             ReLU         52x14x32
conv2  52x14x32    4x4     2       64             ReLU
conv3              3x3     1       64             ReLU
fc4                4x4     2       512            ReLU
fc5                4x4     2       18             Linear

"""

#My simplified Deep Q class
class DeepQ:
    def __init__(self, outputs, memorySize, discountFactor, learningRate, learnStart):
        """
        Parameters:
            - outputs: output size
            - memorySize: size of the memory that will store each state
            - discountFactor: the discount factor (gamma)
            - learningRate: learning rate
            - learnStart: steps to happen before for learning. Set to 128
        """
        self.input_size = inputs
        self.output_size = outputs
        self.memory = memory.Memory(memorySize)
        self.discountFactor = discountFactor
        self.learnStart = learnStart
        self.learningRate = learningRate

    def createModel(self, input_width, input_height):
        self.model = Sequential()
        #normalize the image  to avoid saturation and make the gradients work better
        self.model.add(Lambda(lambda x: x/127.5-1.0, input_shape=INPUT_SHAPE)) #127.5-1.0 = experimental value from udacity self driving car course
        #32 8x8 convolution kernels with 4x4 stride and activation function ReLU
        self.model.add(Conv2D(32, 8, stride=4, activation="relu"))
        self.model.add(Conv2D(64, 4, stride=2, activation="relu"))
        self.model.add(Conv2D(64, 3, stride=1, activation="relu"))
        self.model.add(Flatten())
        self.model.add(Dense(512,activation="relu"))
        self.model.add(Dense(3, activation="linear"))  # 3 outputs for the 3 different actions

        optimizer = optimizers.RMSprop(lr=learningRate, rho=0.9, epsilon=1e-06) # From deepq.py
        self.model.compile(loss="mean_squared_error", optimizer=optimizer)
        self.model.summary()

    #train the network to approximate the bellman equation `r + ymax2a'Q(s',a')`
    #use miniBatch / Experience Replay
    def learn(self, size):
        #X = numpy list of arrays of input data
        #Y = numpy list of arrays of target data
        # Batch size = samples per gradient udpate
        # Do not learn until we've got self.learnStart samples
        if self.memory.getCurrentSize() > self.learnStart:
            # learn in batches of 128
            batch = self.memory.getMiniBatch(size)
            for sample in batch:
                qValues = self.getQValues(state) #model predicted Q(s,a)
                targetValue = self.calculateTarget(sample['newState'], sample['reward'], sample['isFinal'])

                X_batch = np.append(X_batch, np.array([state.copy()]), axis=0) #inuput states with corresponding actions w/ rewards for training
                # We are teaching the network to predict to the discounted reward of taking the optimal action at state s
                Y_sample = qValues.copy()
                Y_sample[sample['action']] = targetValue
                # Every action should be Q(s,a) except for the action taken so that the error on the other action stays 0
                Y_batch = np.append(Y_batch, np.array([Y_sample]), axis=0)
                # X provides the state to feed into the network to calc error based on Y

                #Not sure why this exists???????????????????????
                if sample["isFinal"]:
                    X_batch = np.append(X_batch, np.array([newState.copy()]), axis=0) #Why use new state?
                    #instead of appending discounted reward from bellman equation use final reward
                    Y_batch = np.append(Y_batch, np.array([sample['reward']*3]), axis=0) # 3 = number of output neurons

        self.model.fit(X_batch, Y_batch, batch_size=len(batch), epochs=1, verbose=1)

    def saveModel(self, filepath):
        self.model.save(filepath)

    def loadModel(self, filepath):
        self.model = load_model(filepath)

    def loadWeights(self, filepath):
        self.model.set_weights(load_model(filepath).get_weights())

    def getMaxQ(self, qValues):
        return np.max(qValues)

    # calculate the target function
    def calculateTarget(self, qValuesNewState, reward, isFinal):
        """
        Target = reward(s,a) + gamma * max(Q(s')
        Bellman equation
        """
        if isFinal:
            return reward
        else:
            return reward + self.discountFactor * self.getMaxQ(qValuesNewState)

    # predict Q values for all the actions
    def getQValues(self, state):
        predicted = self.model.prdeict(state)
        return predicted

    # select the action with the highest Q value
    def selectAction(self, qValues, explorationRate): #rate from 0-1
        rand = random.random()
        if rand < explorationRate :
            action = np.random.randint(0, self.output_size)
        else :
            action = self.getMaxIndex(qValues)
        return action

    def addMemory(self, state, action, reward, newState, isFinal):
        self.memory.addMemory(state, action, reward, newState, isFinal)

if __name__ == "__main__":

    rospy.init_node("hal_gym", anonymous=True)
    env = gym.make("HAL-v0")

    print("Open AI gym made")

    env.reset()
    while not rospy.is_shutdown():
        state, reward, done, _ = env.step(0)
        print(reward, done)

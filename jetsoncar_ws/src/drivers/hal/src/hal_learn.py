#!/usr/bin/env python

import hal_env

import gym
import gym_gazebo
import time
import numpy as np
import random
import os
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
from keras.layers import Dense, Flatten, Conv2D, Lambda
from keras.models import load_model
from keras.callbacks import TensorBoard

"""
init replay memory
init action-value function Q with random weights

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
        self.output_size = outputs
        self.memory = Memory(memorySize)
        self.discountFactor = discountFactor
        self.learnStart = learnStart
        self.learningRate = learningRate

        self.model = self.createModel(True)
        self.targetModel = self.createModel() #To add stability to training

    def createModel(self, record=False):
        model = Sequential()
        #normalize the image  to avoid saturation and make the gradients work better
        model.add(Lambda(lambda x: x/127.5-1.0, input_shape=utils.INPUT_SHAPE)) #127.5-1.0 = experimental value from udacity self driving car course
        #32 8x8 convolution kernels with 4x4 stride and activation function ReLU
        model.add(Conv2D(32, 8, strides=4, activation="relu", kernel_initializer='lecun_uniform'))
        model.add(Conv2D(64, 4, strides=2, activation="relu", kernel_initializer='lecun_uniform'))
        model.add(Conv2D(64, 3, strides=1, activation="relu", kernel_initializer='lecun_uniform'))
        model.add(Flatten())
        model.add(Dense(512,activation="relu", kernel_initializer='lecun_uniform'))
        model.add(Dense(3, activation="linear"))  # 3 outputs for the 3 different actions

        optimizer = optimizers.RMSprop(lr=self.learningRate, rho=0.9, epsilon=1e-06) # From deepq.py
        model.compile(loss="mean_squared_error", optimizer=optimizers.Adam(self.learningRate))
        #Try: optimizer=optimizers.Adam(self.learningRate)

        if record:
            timeStamp = time.time()
            path = os.path.dirname(os.path.realpath(__file__)) #get python file path
            self.tensorboard = TensorBoard(log_dir="{}/logs/{}".format(path, timeStamp))
            print("Run `tensorboard --logdir={}/logs/{}` to see CNN status".format(path, timeStamp))
            model.summary()

        return model

    #In order to have a stable training session we must back up a target network so that we can use it to provide a consistent policy at training time
    def backupNetwork(self, model, backup):
        weightMatrix = []
        for layer in model.layers:
            weights = layer.get_weights()
            weightMatrix.append(weights)
        i = 0
        for layer in backup.layers:
            weights = weightMatrix[i]
            layer.set_weights(weights)
            i += 1

    def updateTargetNetwork(self):
        self.backupNetwork(self.model, self.targetModel)
        print("Taget model updated")


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
            X_batch = np.empty((0, utils.INPUT_SHAPE[0], utils.INPUT_SHAPE[1], utils.INPUT_SHAPE[2]), dtype = np.float64)
            Y_batch = np.empty((0, self.output_size), dtype = np.float64)
            for sample in batch:
                state = sample['state']
                qValues = self.getQValues(state) #model predicted Q(s,a)
                qTargetValues = self.getTargetQValues(sample['newState']) #model predicted Q'(s',a')
                targetValue = self.calculateTarget(qTargetValues, sample['reward'], sample['isFinal']) #est. bellman equation

                X_batch = np.append(X_batch, np.array(state.copy()), axis=0) #inuput states with corresponding actions w/ rewards for training
                # We are teaching the network to predict to the discounted reward of taking the optimal action at state s
                Y_sample = qValues.copy()
                Y_sample[0][sample['action']] = targetValue
                # Every action should be Q(s,a) except for the action taken so that the error on the other action stays 0
                Y_batch = np.append(Y_batch, np.array(Y_sample), axis=0)
                # X provides the state to feed into the network to calc error based on Y

                #Not sure why this exists???????????????????????
                if sample["isFinal"]:
                    X_batch = np.append(X_batch, np.array(newState.copy()), axis=0) #Why use new state?
                    #instead of appending discounted reward from bellman equation use final reward
                    Y_batch = np.append(Y_batch, np.full((1,3), sample['reward']), axis=0) # 3 = number of output neurons

            history = self.model.fit(X_batch, Y_batch, batch_size=len(batch), epochs=1, verbose=0, callbacks=[self.tensorboard])
            print("Loss: " + str(history.history['loss']))
            #monitor progress via tensorboard --logdir=logs/hal

    # predict Q values for all the actions
    def getQValues(self, state):
        predicted = self.model.predict(state)
        return predicted

    def getTargetQValues(self, state):
        predicted = self.targetModel.predict(state)
        return predicted

    def saveModel(self, filepath):
        self.model.save(filepath)

    def loadModel(self, filepath):
        self.model = load_model(filepath)

    def loadWeights(self, filepath):
        self.model.set_weights(load_model(filepath).get_weights())

    def getMaxQ(self, qValues):
        return np.argmax(qValues)

    # calculate the target function
    def calculateTarget(self, qValuesNewState, reward, isFinal):
        """
        Target = reward(s,a) + gamma * max(Q(s'))
        Bellman equation
        """
        if isFinal:
            return reward
        else:
            return reward + self.discountFactor * self.getMaxQ(qValuesNewState)
            #`self.discountFactor * self.getMaxQ(qValuesNewState)` is an approximation but will improve as the network is trained

    # select the action with the highest Q value
    def selectAction(self, qValues, explorationRate): #rate from 0-1
        rand = random.random()
        if rand < explorationRate:
            action = np.random.randint(0, self.output_size)
        else:
            action = self.getMaxQ(qValues)
        return action

    def addMemory(self, state, action, reward, newState, isFinal):
        self.memory.addMemory(state, action, reward, newState, isFinal)

if __name__ == "__main__":

    rospy.init_node("hal_gym", anonymous=True)
    env = gym.make("HAL-v0")

    print("Open AI gym made")

    #Set starting state varaibles that will influence learning process
    totalEpisodes = 2000
    updateTargetNetwork = 10000 #number of steps until the target network is updated
    timeStepLimit = 5000
    explorationRate = 0.9 #starting epsilon
    epsilon_discount = 0.999 #2301 steps to reach 0.1 w/ starting rate = 1 -> ln(0.1)/(ln(discount)*starting _exploration_rate)
    learning_size = 128 #how many memory samples to train on per step

    print("Running %d episodes with up to %d time steps..." % (totalEpisodes, timeStepLimit))

    deepq = DeepQ(outputs=env.action_space.n, memorySize=4096, discountFactor=0.9, learningRate=1.0e-4, learnStart=128)

    highest_cumulative_reward = 0
    stepCounter = 0
    for i in range(totalEpisodes):
        if stepCounter > updateTargetNetwork:
            stepCounter = 0
            deepq.updateTargetNetwork()
        cumulative_reward = 0
        state = env.reset()
        print ("Episode = "+str(i))

        #decay epsilon exploration rate
        if explorationRate > 0.05:
            explorationRate *= epsilon_discount

        for x in range(timeStepLimit):
            #print(deepq.getQValues(state))
            #print(np.argmax(deepq.getQValues(state)))
            action = deepq.selectAction(deepq.getQValues(state), explorationRate)

            #print("action = " + str(action))
            newState, reward, done, info = env.step(action)

            #print("reward = " + str(reward) + "\n")
            cumulative_reward += reward

            deepq.addMemory(state, action, reward, newState, done)

            stepCounter += 1

            if not(done):
                state = newState
            else:
                deepq.learn(learning_size)
                if(cumulative_reward > highest_cumulative_reward):
                    highest_cumulative_reward = cumulative_reward
                print("Episode finished with cumulative reward: " + str(cumulative_reward) + " and exploration rate: " + str(explorationRate) + "\n")
                break

    print("Training done\nHighest cumulative reward = " + str(highest_cumulative_reward))
    deepq.saveModel("hal_" + str(highest_cumulative_reward) + ".h5")

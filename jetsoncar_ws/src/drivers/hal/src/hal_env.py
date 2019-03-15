#!/usr/bin/env python

#Open AI Gym Enviroment for the autonomous car

import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from std_srvs.srv import Empty
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu, Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import utils

from gym.utils import seeding

from gym.envs.registration import register

reg = register(
    id="HAL-v0",
    entry_point="hal_env:HALenv",
    timestep_limit=5000,
    )

class HALenv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name

        self.ackermann_pub = rospy.Publisher('/racecar/autonomous/ackermann_cmd', AckermannDriveStamped, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Discrete(3) #Go Strait, Turn Left, Turn Right
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        #input action : return new state, reward, done, and info
        #define what done is here
        #unpause and pause physics while taking the step


        #image preproccessing
        # image = utils.crop(image)
        # image = utils.resize(image)
        # image = utils.rgb2yuv(image)


        #unpause
        #take action
        #new state
        #compute reward and check if DONE

        state = np.zeros(utils.IMAGE_HEIGHT, utils.IMAGE_WIDTH, utils.IMAGE_CHANNELS*4, dtype=int)

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException, e:
            print ("/gazebo/unpause_physics service call failed")

        ackermann_cmd = AckermannDriveStamped()

        if action == 0: #FORWARD
            ackermann_cmd.speed = 6
            ackermann_cmd.steering_angle = 0.0

        elif action == 1: #LEFT
            ackermann_cmd.speed = 6
            ackermann_cmd.steering_angle = 0.5

        elif action == 2: #RIGHT
            ackermann_cmd.speed = 6
            ackermann_cmd.steering_angle = -0.5

        self.ackermann_pub.publish(ackermann_cmd)

        cameraData = None
        #state array contains 4 RGB images stacked in the 3rd dimension
        for i in range(4):
            while cameraData is None:
                try:
                    cameraData = rospy.wait_for_message('/front_cam/color/image_raw', Image, timeout=2)
                    image = utils.rgb2yuv(utils.resize(utils.crop(cameraData)))
                    for rgbLayer in range(3):
                        state[:, :, 4*i-rgbLayer] = image[:, :, rgbLayer] #;)
                except:
                    pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException, e:
            print ("/gazebo/pause_physics service call failed")

        #determine if the episode is done
        #for track 1

        if not done:
            if action == 0:
                reward = 8
            elif action == 0:
                reward = 1
            else:
                reward = 2
        else:
            reward = -200

        return state, reward, done, {}

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        # Move car to a new random starting location in the valid track area
        # Adjust lighting randomly for domain randomization
        # Add obstacles randomly??? -> will need to add in depth point cloud which may make nn to big

        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except rospy.ServiceException, e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException, e:
            print ("/gazebo/unpause_physics service call failed")

        #read camera data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/front_cam/color/image_raw', Image, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException, e:
            print ("/gazebo/pause_physics service call failed")

        state = self.discretize_observation(cameraData)

        return state

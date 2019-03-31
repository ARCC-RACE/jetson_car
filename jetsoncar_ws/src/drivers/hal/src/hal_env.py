#!/usr/bin/env python

#Open AI Gym Enviroment for the autonomous car

import gym
import rospy
import roslaunch
import rospkg
import time
import numpy as np
import math

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from std_srvs.srv import Empty
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, Image
from gazebo_msgs.msg import ModelStates
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

def dist(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

class HALenv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        #For respawning the simulation if it crashes
        self.launch = roslaunch.scriptapi.ROSLaunch()

        self.autonomous_mode_pub = rospy.Publisher('racecar/autonomous_mode', Bool, queue_size=2)
        self.ackermann_pub = rospy.Publisher('/racecar/autonomous/ackermann_cmd', AckermannDriveStamped, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Discrete(3) #Go Strait, Turn Left, Turn Right
        self.reward_range = (-np.inf, np.inf)

        self.lastPose = self._getModelPose()
        self.lastStepTime = rospy.get_time()

        # two different targets at each end of the track
        # once one target is hit the target will switch to the next one
        # target 0 is on the far side of the track while target 1 is near the starting location
        # target range should be x+-4.5
        self.targets = [[174,0],[6,0]]
        self.currentTarget = 0

        #self.state = np.zeros((1, utils.IMAGE_HEIGHT, utils.IMAGE_WIDTH, utils.IMAGE_CHANNELS), dtype=int)
        #4D required for keras conv net

        self._seed()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _getModelPose(self):
        modelState = None
        while modelState is None:
            try:
                modelState = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
                modelStateIndex = 0
                for i in range(len(modelState.name)):
                    if modelState.name[i] == "racecar":
                        modelStateIndex = i
                        break
            except:
                #If this fails there is a good chance that gazebo restarted, launching a new car fixes this
                print("Model pose timed out")
                self._respawnCar()

        return modelState.pose[modelStateIndex].position.x, modelState.pose[modelStateIndex].position.y

    #Start or resurect a simulation instance
    def _respawnCar(self):

        #Stop the last crashed simulation
        try: #prevents declaration issue at start of system
            self.launch.stop()
        except:
            pass

        time.sleep(10) #wait for system to stop cleanly

        r = rospkg.RosPack()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [r.get_path('racecar_worlds') + "/launch/track_1_world_no_gui.launch", r.get_path('racecar_description') + "/launch/spawn_racecar.launch"] )
        self.launch.start() #start simulation and spawn the car

        time.sleep(10) #wait for system to start cleanly

        self.autonomous_mode_pub.publish(True) #Put system into autonomous mode without need for teleop interface (node)


    def _stopCar(self):
        ackermann_cmd = AckermannDriveStamped()
        self.ackermann_pub.publish(ackermann_cmd)

    def _updateState(self):
        cameraData = None
        bridge = CvBridge()

        #get the state variable setup
        try:
            cameraData = rospy.wait_for_message('/front_cam/color/image_raw', Image, timeout=5)
            image = bridge.imgmsg_to_cv2(cameraData, desired_encoding="bgr8")
            state = np.expand_dims(utils.preprocess(image), axis=0) #4D for keras input
        except:
            #If this fails there is a good chance that gazebo restarted, launching a new car fixes this
            print("Camera timed out")
            self._respawnCar()

        #Compare the number of channels of the state array with that of a stack of 4 images
        while state.shape[3] < utils.INPUT_SHAPE[2]:
            try:
                cameraData = rospy.wait_for_message('/front_cam/color/image_raw', Image, timeout=5)
                image = utils.preprocess(bridge.imgmsg_to_cv2(cameraData, desired_encoding="bgr8"))
                state = np.append(state, [image], axis=3)
            except:
                #If this fails there is a good chance that gazebo restarted, launching a new car fixes this
                print("Camera timed out")
                self._respawnCar()
        self.state = state

    def step(self, action):
        #input action : return new state, reward, done, and info
        #define what done is here
        #unpause and pause physics while taking the step

        # image preproccessing
        # image = utils.crop(image)
        # image = utils.resize(image)
        # image = utils.rgb2yuv(image)

        #unpause
        #take action
        #new state
        #compute reward and check if DONE

        rospy.wait_for_service('/gazebo/unpause_physics', timeout=5)
        #try:
        self.unpause()
        #except rospy.ServiceException, e:
        #    print ("/gazebo/unpause_physics service call failed")

        ackermann_cmd = AckermannDriveStamped()

        #Make this non-discrete in the future
        if action == 0: #FORWARD
            ackermann_cmd.drive.speed = 6
            ackermann_cmd.drive.steering_angle = 0.0

        elif action == 1: #LEFT
            ackermann_cmd.drive.speed = 6
            ackermann_cmd.drive.steering_angle = 0.5

        elif action == 2: #RIGHT
            ackermann_cmd.drive.speed = 6
            ackermann_cmd.drive.steering_angle = -0.5

        self.ackermann_pub.publish(ackermann_cmd)

        self._updateState()

        pose = self._getModelPose()
        #pose[0] = x, pose[1] = y

        stepTime = rospy.get_time()

        rospy.wait_for_service('/gazebo/pause_physics', timeout=5)
        #try:
        self.pause()
        #except rospy.ServiceException, e:
        #    print ("/gazebo/pause_physics service call failed")

        # determine if the episode is done
        # for track 1
        # X and Y bounds defined below
        #(x-offset)^2+y^2=r^2
        done = False
        if pose[1] > 49 or pose[1] < -49:
            done = True
        if pose[0] > 44 and pose[0] < 125: #if the car is in the area between turns and out of bounds
            if pose[1] < 0 and pose[1] > -35:  #on left side
                done = True
            elif pose[1] > 0 and pose[1] < 35: #on right side
                done = True
        #       ro          x      offset    y              ri          x       offset   y               Make sure that the car is in the bounds of the turn
        if not (49**2 > ((pose[0]-44.5)**2+pose[1]**2) and 37**2 < ((pose[0]-44.5)**2+pose[1]**2)) and pose[0] < 44:    #Upper turn
            done = True
        if not (49**2 > ((pose[0]-125.6)**2+pose[1]**2) and 37**2 < ((pose[0]-125.6)**2+pose[1]**2)) and pose[0] > 125: #Lower turn
            done = True


        if not done:
            #dz/dt: change in distance / change in time to compute reward (want to get closer to target faster)
            if not stepTime-self.lastStepTime == 0: # make sure we dont do any dividing by zero
                reward = ((dist(self.lastPose[0], self.lastPose[1], self.targets[self.currentTarget][0], self.targets[self.currentTarget][1])-
                            dist(pose[0], pose[1], self.targets[self.currentTarget][0], self.targets[self.currentTarget][1]))/(stepTime-self.lastStepTime))
            else:
                reward = 0

            #switch targets if needed
            #When the sign changes on pose then power*lastPose will be negative
            if (pose[1]-2)*(self.lastPose[1]-2) < 0:
                #85 = minor axis of symmetry
                if pose[0] > 85 and self.currentTarget == 0:
                    reward = 500
                    self.currentTarget = 1 #switch reward to top
                elif pose[1] < 85 and self.currentTarget == 1:
                    reward = 500
                    self.currentTarget = 0 #switch reward to bottom

            self.lastPose = pose
            self.lastStepTime = stepTime
        else:
            reward = -200

        return self.state, reward, done, {}

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        # Move car to a new random starting location in the valid track area
        # Adjust lighting randomly for domain randomization
        # Add obstacles randomly??? -> will need to add in depth point cloud which may make nn to big

        self._stopCar()

        rospy.wait_for_service('/gazebo/reset_simulation', timeout=5)
        #try:
        self.reset_proxy()
        #except rospy.ServiceException, e:
        #    print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics', timeout=5)
        #try:
        self.unpause()
        #except rospy.ServiceException, e:
        #    print ("/gazebo/unpause_physics service call failed")

        time.sleep(1) #Wait for simulation to fully reset

        self._updateState()

        rospy.wait_for_service('/gazebo/pause_physics', timeout=5)
        #try:
        self.pause()
        #except rospy.ServiceException, e:
        #    print ("/gazebo/pause_physics service call failed")

        return self.state

#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

# The joystick will auto repeat values at 2Hz so we can start a watchdog timer

# Xbox One Controller BUTTON MAPPING
# X [2]  = Flip steering                                             flipSteering
# Y [3]  = Fine steering mode (-1 to 1 -> -0.5 to 0.5)               fineSteering
# A [0]  = Switch driving mode (default = RT+LT, alternate = RT-LT)  drivingMode
# B [1]  = No function                                               B4
# LB [4] = Brake - NOT WORKING
# RB [5] = Brake - NOT WORKING

# Xbox One Controller AXIS MAPPING
# LT [2] = throttle forward/reverse (-1 to 1)  throttleFR
# RT [5] = throttle forward (-1 to 1)          throttleF
# L [0]  = steering control (-1 to 1)          steeringLR
# L [1]  = No function                         A4

# DS4 Controller BUTTON MAPPING
# Square [3]  = Flip steering                                            flipSteering
# Triangle [2]  = Fine steering mode (-1 to 1 -> -0.5 to 0.5)            fineSteering
# Cross [0]  = Switch driving mode (default = RT+LT, alternate = RT-LT)  drivingMode
# Circle [1]  = No function                                              B4

# DS4 Controller AXIS MAPPING
# L2 [2] = throttle forward/reverse (-1 to 1)  throttleFR
# R2 [5] = throttle forward (-1 to 1)          throttleF
# L [0]  = steering control (-1 to 1)          steeringLR
# L [1]  = No function                         A4

rospy.init_node("joy_teleop")
controllerMap = rospy.get_param("~controller_mapping")  # "xbox" or "ds4"

# default = ds4
if controllerMap == "xbox":
    rospy.loginfo("Using xbox one controller mapping")
    throttleFR = 2
    throttleF = 5
    steeringLR = 0
    A4 = 1
    flipSteering = 2
    fineSteering = 3
    drivingMode = 0
    B4 = 1
else:
    rospy.loginfo("Using dual shock 4 (ds4) controller mapping")
    throttleFR = 2
    throttleF = 5
    steeringLR = 0
    A4 = 1
    flipSteering = 3
    fineSteering = 2
    drivingMode = 0
    B4 = 1

# deals with LT RT starting at zero by waiting to write thrusters until both axes have been updated
controllerInitializationStarted = False  # allows for two step start (LT RT to 1 and then back to 0)
controllerInitialized = False

# global vars with default values set
# vars for flipping steering
lastSteeringState = 0  # in order to look for rising edge on button
steeringState = -1

# vars for switching driving mode
lastThrottleState = 0  # in order to look for rising edge on button
throttleState = True  # true means adding throttles (no reverse), false means reverse enabled

lastFineSteeringState = 0
fineSteeringState = True  # true means -1 to 1, false means -0.5 to 0.5 (rads in simulation)

# for watchdog
lastUpdate = 0


# helper function to map values
def map(input, inMin, inMax, outMin, outMax):
    output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
    return output


def JoyCB(data):
    # Use global/static vars
    global controllerInitializationStarted
    global controllerInitialized

    global lastSteeringState
    global steeringState

    global lastThrottleState
    global throttleState

    global lastFineSteeringState
    global fineSteeringState

    # For watchdog timer
    global lastUpdate
    lastUpdate = rospy.get_time()

    # steering direction
    if data.buttons[flipSteering] != lastSteeringState and lastSteeringState == 0:
        steeringState = -steeringState
        rospy.logdebug("Changing steering direction")
    lastSteeringState = data.buttons[flipSteering]

    # driving mode
    if data.buttons[drivingMode] != lastThrottleState and lastThrottleState == 0:
        throttleState = not throttleState
        rospy.logdebug("Changing driving mode")
    lastThrottleState = data.buttons[drivingMode]

    # fine steering mode
    if data.buttons[fineSteering] != lastFineSteeringState and lastFineSteeringState == 0:
        fineSteeringState = not fineSteeringState
        rospy.logdebug("Changing steering mode")
    lastFineSteeringState = data.buttons[fineSteering]

    # apply modificaitons to throttle based on drive mode selection
    # acceptible values for physical jetson car are +-5 (not in m/s) for now
    if not controllerInitialized:
        throttleVal = 0
        if (controllerInitializationStarted and data.axes[throttleFR] == 1 and data.axes[
            throttleF] == 1):  # controller should be corrected by now
            controllerInitialized = True
        elif data.axes[throttleFR] == -1 and data.axes[throttleF] == -1:
            controllerInitializationStarted = True
    elif throttleState:
        throttleVal = map(data.axes[throttleF], 1, -1, 0, 2) + map(data.axes[throttleFR], 1, -1, 0,
                                                                   2)  # range is 0 to 4
    else:
        throttleVal = map(data.axes[throttleF], 1, -1, 0, 2) - map(data.axes[throttleFR], 1, -1, 0,
                                                                   2)  # range is -2 to 2

    #    #handle braking if applied (doesn't correlate to anything in simulation)
    #    if(data.buttons[4] == 1 or data.buttons[5] == 1):
    #        throttleVal = 6 #out of range value triggers active brake in ARC1 firmware

    # apply modifications to steering based on steering mode selection
    if fineSteeringState:
        steeringVal = data.axes[steeringLR]  # range is -1 to 1
    else:
        steeringVal = data.axes[steeringLR] / 2  # range is -0.5 to 0.5

    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.drive.acceleration = 0  # drive acceleration unknown
    msg.drive.speed = throttleVal
    msg.drive.jerk = 0  # jerk unknown
    msg.drive.steering_angle = steeringVal * steeringState  # steering state is 1 or -1 to flip steering if needed
    msg.drive.steering_angle_velocity = 0  # angle velocity unknown

    pub.publish(msg)


def watchdogTimer(event):
    global lastUpdate
    if rospy.get_time() - lastUpdate > 0.15:
        msg = AckermannDriveStamped();
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        pub.publish(msg)


rospy.Subscriber("joy", Joy, JoyCB)
pub = rospy.Publisher('controller/ackermann_cmd', AckermannDriveStamped, queue_size=1)
rospy.Timer(rospy.Duration(0.15), watchdogTimer)  # duration time = 1/Hz (added some leeway for the system)

rospy.spin()  # keep node from closing

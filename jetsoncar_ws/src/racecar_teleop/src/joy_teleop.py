#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

#BUTTON MAPPING
# X [2]  = Flip steering
# Y [3]  = Fine steering mode (-1 to 1 -> -0.5 to 0.5)
# A [0]  = Switch driving mode (default = RT+LT, alternate = RT-LT)
# B [1]  = No function
# LB [4] = Brake
# RB [5] = Brake

#AXIS MAPPING
# LT [2] = throttle forward/reverse (-1 to 1)
# RT [5] = throttle forward (-1 to 1)
# L [0]  = steering control (-1 to 1)
# L [1]  = No function

#global vars with default values set
#vars for flipping steering
lastSteeringState = 0  #in order to look for rising edge on button
steeringState = -1

#vars for switching driving mode
lastThrottleState = 0 #in order to look for rising edge on button
throttleState = True  #true means adding throttles (no reverse), false means reverse enabled

lastFineSteeringState = 0
fineSteeringState = True #true means -1 to 1, false means -0.5 to 0.5 (rads in simulation)

#helper function to map values
def map(input, inMin, inMax, outMin, outMax):
    output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
    return output

def JoyCB(data):

    #Use global/static vars
    global lastSteeringState
    global steeringState

    global lastThrottleState
    global throttleState

    global lastFineSteeringState
    global fineSteeringState

    #steering direction
    if(data.buttons[2] != lastSteeringState and lastSteeringState == 0):
         steeringState = -steeringState
         rospy.logdebug("Changing steering direction")
    lastSteeringState = data.buttons[2]

    #driving mode
    if(data.buttons[0] != lastThrottleState and lastThrottleState == 0):
         throttleState = not throttleState
         rospy.logdebug("Changing driving mode")
    lastThrottleState = data.buttons[0]

    #fine steering mode
    if(data.buttons[3] != lastFineSteeringState and lastFineSteeringState == 0):
        fineSteeringState = not fineSteeringState
        rospy.logdebug("Changing steering mode")
    lastFineSteeringState = data.buttons[3]

    #apply modificaitons to throttle based on drive mode selection
    if throttleState:
         throttleVal = map(data.axes[5], 1, -1, 0, 2) + map(data.axes[2], 1, -1, 0, 2) #range is 0 to 4
    else:
         throttleVal = map(data.axes[5], 1, -1, 0, 2) - map(data.axes[2], 1, -1, 0, 2) #range is -2 to 2

    #apply modifications to steering based on steering mode selection
    if fineSteeringState:
        steeringVal = data.axes[0]   #range is -1 to 1
    else:
        steeringVal = data.axes[0]/2 #range is -0.5 to 0.5

    msg = AckermannDriveStamped();
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.drive.acceleration = 0; #drive acceleration unknown
    msg.drive.speed = throttleVal
    msg.drive.jerk = 0; #jerk unknown
    msg.drive.steering_angle = steeringVal*steeringState #steering state is 1 or -1 to flip steering if needed
    msg.drive.steering_angle_velocity = 0 #angle velocity unknown

    pub.publish(msg)

rospy.init_node("joy_teleop")
rospy.Subscriber("joy", Joy, JoyCB)
pub = rospy.Publisher('/racecar/ackermann_cmd', AckermannDriveStamped, queue_size=1)

rospy.spin() #keep node from closing

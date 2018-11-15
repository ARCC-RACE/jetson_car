#!/usr/bin/env python

import serial
import rospy 
from control_msgs.msg import JointControllerState

steeringVal = 0
throttleVal = 0

def intMap(value, inMin, inMax, outMin, outMax):
  return(((value-inMin)/(inMax-inMin)*(outMax-outMin))+outMin)

def updateSteering(data):
   global steeringVal
   steeringVal = intMap(data.process_value, -1, 1, 1100, 1900)

def updateThrottle(data):
   global throttleVal
   throttleVal = intMap(data.process_value, 0, 100, 1500, 1900)

def formPacket(throttle, steering):
   packet = (unichr((int(throttle) & 0b1111111100000000)), (unichr(int(throttle) & 0b0000000011111111)), (unichr(int(steering) & 0b1111111100000000)), (unichr(int(steering) & 0b0000000011111111)))

   return packet

arduinoSer = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
   
rospy.init_node("hardware_controller")
SteeringSub = rospy.Subscriber("racecar/left_steering_hinge_position_controller/state", JointControllerState, updateSteering)
throttleSub = rospy.Subscriber("racecar/left_rear_wheel_velocity_controller/state", JointControllerState, updateThrottle)
rate = rospy.Rate(30)

while not rospy.is_shutdown():
   rospy.loginfo(steeringVal)
   arduinoSer.write(formPacket(throttleVal, steeringVal))
   rate.sleep()
      

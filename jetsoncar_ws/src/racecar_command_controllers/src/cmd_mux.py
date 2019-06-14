#!/usr/bin/env python

#takes in desired commands from autonomous system and joystick and determines the output

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

#mux matrix
#autonomous mode    YES                                NO
#sport_mode  NO  connect to nn and use cruise speed    use cruise speed
#           YES  connect to nn and use control speed   use all controller inputs

isAutonomous = False
isSport = True #basically controller input
cruiseVel = 2 #not correlative to any real wold value yet

autonomousMsg = AckermannDriveStamped()
controllerMsg = AckermannDriveStamped()

#NOTE Currently SPORT MODE does not do anything!!!!!

#update control interface inputs
def sportMode(data):
    global isSport
    isSport = data.data
def cruiseVel(data):
    global cruiseVel
    cruiseVel = data.data
def autonomousMode(data):
    global isAutonomous
    isAutonomous = data.data

#update drive control inputs
def controllerAckermann(data):
    global controllerMsg
    controllerMsg = data

    #mux matrix
    # if isSport and not isAutonomous:
    #     throttleVal = data.drive.speed
    #     steeringVal = data.drive.steering_angle
    # elif isSport and isAutonomous:
    #     throttleVal = data.drive.speed
    # elif not isSport and not isAutonomous:
    #     pass
    # else:
    #     throttleVal = 0
    #     steeringVal = 0

def autonomousAckermann(data):
    global autonomousMsg
    autonomousMsg = data

steering_trim = 0 #default steering trim is zero
def steeringTrim(data):
    global steering_trim
    steering_trim = data.data



rospy.init_node("cmd_mux")

#mode selectors from the control interface
rospy.Subscriber("/racecar/autonomous_mode", Bool, autonomousMode)
rospy.Subscriber("/racecar/cruise_velocity", Float64, cruiseVel)
rospy.Subscriber("/racecar/sport_mode", Bool, sportMode)

#input drive control topics
rospy.Subscriber("/racecar/controller/ackermann_cmd", AckermannDriveStamped, controllerAckermann)
rospy.Subscriber("/racecar/autonomous/ackermann_cmd", AckermannDriveStamped, autonomousAckermann)
rospy.Subscriber("/racecar/steering_trim", Float64, steeringTrim)


pub = rospy.Publisher('/racecar/muxed/ackermann_cmd', AckermannDriveStamped, queue_size=1)

rate = rospy.Rate(50)
while not rospy.is_shutdown():
    if isAutonomous:
        msg = autonomousMsg
    else:
        msg = controllerMsg
    msg.drive.steering_angle = msg.drive.steering_angle + steering_trim
    # msg = AckermannDriveStamped();
    # msg.header.stamp = rospy.Time.now()
    # msg.header.frame_id = "base_link"
    #
    # msg.drive.acceleration = 0; #drive acceleration unknown
    # msg.drive.speed = throttleVal
    # msg.drive.jerk = 0; #jerk unknown
    # msg.drive.steering_angle = steeringVal
    # msg.drive.steering_angle_velocity = 0 #angle velocity unknown

    pub.publish(msg)

    #Catch time going backwards error that occurs when Gazebo simulation is reset
    try:
        rate.sleep()
    except rospy.ROSInterruptException:
        pass

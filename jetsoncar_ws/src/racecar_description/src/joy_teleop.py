#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

def JoyCB(data):

    msg = AckermannDriveStamped();
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.drive.speed = data.axes[1]/5 #x/5 makes max throttle 2/10
    msg.drive.acceleration = 1;
    msg.drive.jerk = 1;
    msg.drive.steering_angle = data.axes[0]
    msg.drive.steering_angle_velocity = 1

    pub.publish(msg)

rospy.init_node("joy_teleop")
rospy.Subscriber("joy", Joy, JoyCB)
pub = rospy.Publisher('/racecar/ackermann_cmd', AckermannDriveStamped, queue_size=1)

rospy.spin()

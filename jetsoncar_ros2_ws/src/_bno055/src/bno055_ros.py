#!/usr/bin/env python

import math

import BNO055

import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from bno055.msg import Bno055


class Bno055Publisher(Node):
    def __init__(self):
        super().__init__('imu')
        self.imu = self.create_publisher(Imu, 'racecar/imu', 3)
        self.imuInfo = self.create_pulisher(Bno055, 'racecar/imu_info', 3)
        sensor = BNO055.BNO055()
        self.initializeBno055(sensor)
        self.readSystemStatus(sensor)
        self.diagnosticData(sensor)

    def initializeBno055(self, sensor):
        attempts = 0
        while attempts < 10:
            try:
                sensor.begin()
                break
            except Exception as e:
                rclpy.get_logger().info('Failed to initialize BNO055! %s', e)
                attempts += 1
                time.sleep(0.25)

        if attempts == 10:
            rclpy.get_logger().info('Failed to Initialize IMU, exiting!')
            exit(1)

    def readSystemStatus(self, sensor):
        try:
            status, self_test, error = sensor.get_system_status()
        except Exception as e:
            rclpy.get_logger().info('Failed to read BNO055 system status! %s', e)

        rclpy.get_logger().info('System status: %s', status)
        rclpy.get_logger().info('Self test result (0x0F is normal): %s', hex(self_test))

        if status == 0x01:
            rclpy.get_logger().info('System error: %s', error)
            rclpy.get_logger().info('See datasheet section 4.3.59 for the meaning.')

    def diagnosticData(self, sensor):
        try:
            sw, bl, accel, mag, gyro = sensor.get_revision()
        except Exception as e:
            rclpy.get_logger().info('Failed to read BNO055 meta-inforamtion! %s', e)

        rclpy.get_logger().info('Software version:   %d', sw)
        rclpy.get_logger().info('Bootloader version: %d', bl)
        rclpy.get_logger().info('Accelerometer ID:   %s', hex(accel))
        rclpy.get_logger().info('Magnetometer ID:    %s', hex(mag))
        rclpy.get_logger().info('Gyroscope ID:       %s', hex(gyro))

        rclpy.get_logger().info('Reading BNO055 data...')

    def readData(self, sensor):
        # Define messages
        msgHeader = Header()
        infoHeader = Header()
        msg = Imu()
        info = bno055_info()

        orientation = Quaternion()
        angular_vel = Vector3()
        linear_accel = Vector3()
        attempts = 0
        while attempts < 4:
            try:
                # Orientation as a quaternion:
                orientation.x, orientation.y, orientation.z, orientation.w = sensor.read_quaternion()

                # Gyroscope data (in degrees per second converted to radians per second):
                gry_x, gry_y, gry_z = sensor.read_gyroscope()
                angular_vel.x = math.radians(gry_x)
                angular_vel.y = math.radians(gry_y)
                angular_vel.z = math.radians(gry_z)

                # Linear acceleration data (i.e. acceleration from movement, not gravity--
                # returned in meters per second squared):
                linear_accel.x, linear_accel.y, linear_accel.z = sensor.read_linear_acceleration()
                break
            except Exception as e:
                rclpy.get_logger().info('Failed to read data! %s', e)
                attempts += 1
                time.sleep(.01)
        if attempts != 4:
            msg.orientation = orientation
            msg.angular_velocity = angular_vel
            msg.linear_acceleration = linear_accel

        # Update mesage headers
        msgHeader.stamp = time.localtime()
        msgHeader.frame_id = 'imu_link'
        msg.header = msgHeader
        infoHeader.frame_id = 'imu_info'
        info.header = infoHeader
        self.imu.publish(msg)
        self.imuInfo.publish(info)


def main(args=None):
    rclpy.init(args=args)

    bno055Publisher = Bno055Publisher()

    rclpy.spin(bno055Publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bno055Publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# Unused functions
# Read the Euler angles for heading, roll, pitch (all in degrees).
# heading, roll, pitch = sensor.read_euler()
# Magnetometer data (in micro-Teslas):
# x,y,z = sensor.read_magnetometer()
# Accelerometer data (in meters per second squared):
# x,y,z = sensor.read_accelerometer()
# Gravity acceleration data (i.e. acceleration just from gravity--returned
# in meters per second squared):
# x,y,z = sensor.read_gravity()

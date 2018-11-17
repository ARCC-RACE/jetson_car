#ifndef RACECAR_ARDUINO_COM_H
#define RACECAR_ARDUINO_COM_H

#include <unistd.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <stdint.h>
#include <ros/ros.h>

#include "serial/serial.h"


enum Joint // Corresponds the the index in the arrays passed in by hardware_interface
{
  LEFT_REAR_WHEEL,     // Starting at 0
  RIGHT_REAR_WHEEL,
  LEFT_FRONT_WHEEL,
  RIGHT_FRONT_WHEEL,
  LEFT_STEERING_HINGE,
  RIGHT_STEERING_HINGE,
};


class HardwareCom
{
  //Serial object that manages USB serial connection
  serial::Serial connection;

  //Packet structure = |PEC|steering MSB|steering LSB|throttle MSB|throttle LSB|
  uint8_t packetDown[5];        //Packet to the arduino (length = 4 + PEC)
  const int packetDownSize = 5; //number of bytes of data + PEC byte

  uint8_t packetUp[4];      //Packet from the arduino (length = 4) (unused)
  double steeringPosition;  //Member var for holding "actual" steering angle
  double wheelVelocity;     //Member var for holding "actual" wheel velocity


public:
  HardwareCom(std::string port, int baud);     //Constructor
  void setController(int length, double* cmd); //Sends input commands to arduino
  double readController(Joint joint);          //Returns data read from the arduino

};

#endif

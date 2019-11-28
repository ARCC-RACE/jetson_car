#ifndef RACECAR_ARDUINO_COM_H
#define RACECAR_ARDUINO_COM_H

#include <unistd.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdint>
#include <bitset>

#include "serial/serial.h"

class HardwareCom
{
  //Serial object that manages USB serial connection
  serial::Serial connection;
  //Packet structure = |PEC|steering MSB|steering LSB|throttle MSB|throttle LSB|
  const int outgoingPacketLength = 5;
  uint8_t outgoingPacket[5];   //Packet to the arduino (length = 4 + PEC)
  //Packet structure = |ch1 LSB|ch1 MSB|ch2 LSB|ch2 MSB|ch3 LSB|ch3 MSB|ch4 LSB|ch4 MSB|ch5 LSB|ch5 MSB|ch6 LSB|ch6 MSB|PEC| <- may be reversed
  const int incomingPacketLength = 13;
  uint8_t incomingPacket[13]; //Packet from the zero (length = 12 + PEC)
  int channels[6] = {}; //Values from CH1 to Ch6


public:
  HardwareCom(std::string port, int baud);            //Constructor
  bool setController(double steeringCmd, double throttleCmd); //Sends input commands to arduino zero
  bool readController();                            //Returns data read from the arduino zero
  int getCh1();
  int getCh2();
  int getCh3();
  int getCh4();
  int getCh5();
  int getCh6();
};

#endif

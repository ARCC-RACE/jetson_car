#include <racecar_hardware_interface/racecar_arduino_com.h>

/**
* @breif constrians value between min and max inclusive. Value is returned by reference.
* @param[in,out] value input to be constrianed
* @param[in] min The minimum value that "value" should be able to be
* @param[in] max The maximum value that "value" should be able to be
*/
template <class T>
void constrain(T &value, T min, T max){
    if(value > max){
        value = max;
    } else if(value < min){
        value = min;
    }
}

/**
* @breif returns a number mapped proportioanlly from one range of numbers to another
* @param[in] input Value to be mapped
* @param[in] inMax The maximum value for the range of the input
* @param[in] inMin The minimum value for the range of the input
* @param[in] outMin The minimum value for the range of the output
* @param[in] outMax The maximum value for the range of the output
* @return The input trnslated proportionally from range in to range out
*/
template <class T>
T map(T input, T inMin, T inMax, T outMin, T outMax){
    T output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    return output;
}


HardwareCom::HardwareCom(std::string port, int baud): connection(port, baud, serial::Timeout::simpleTimeout(10),
                                                                 serial::eightbits, serial::parity_even, serial::stopbits_one)
//Serial set to SER_8E1 (8 bit, even parity, 1 stop bit)
{
  //open serial port
  if(connection.isOpen()){
    std::cout << "Port " << port << " opened successfully!\n";
  } else {
    std::cerr << "Port " << port << " failed to open successfully!\n";
    exit(1);
  }
}

bool HardwareCom::setController(double steeringCmd, double throttleCmd){
  std::cout << "Steering cmd: " << steeringCmd << "\n";
  std::cout << "Throttle cmd: " << throttleCmd << "\n";
  auto steeringVal = static_cast<int>(map<double>(steeringCmd, -1, 1, -1000, 1000));
  auto throttleVal = static_cast<int>(map<double>(throttleCmd, -1, 1, -1000, 1000));
  constrain(steeringVal, -1000, 1000);

  //Safety precausion if throttle value goes out of bounds
  if(throttleVal > 1000 || throttleVal < -1000){
    std::cerr << "Throttle value out of bounds!\n";
    throttleVal = 0;
  }

  //Fill outGoingPacket with four bytes with proper packet structure
    outgoingPacket[3] = static_cast<uint8_t>(steeringVal >> 8);                 //Steering MSB
    outgoingPacket[2] = static_cast<uint8_t>(steeringVal & 0b0000000011111111); //Steering LSB
    outgoingPacket[1] = static_cast<uint8_t>(throttleVal >> 8);                 //Throttle  MSB
    outgoingPacket[0] = static_cast<uint8_t>(throttleVal & 0b0000000011111111); //Throttle LSB

    outgoingPacket[4] = static_cast<uint8_t>(outgoingPacket[0]^outgoingPacket[1]^outgoingPacket[2]^outgoingPacket[3]); //PEC by XORing all values

  std::cout << "packet down: " <<  std::bitset<8>(outgoingPacket[3]) << std::bitset<8>(outgoingPacket[2]) << std::bitset<8>(outgoingPacket[1]) << std::bitset<8>(outgoingPacket[0]) << "\n";

  size_t bytesSent = connection.write(outgoingPacket, outgoingPacketLength); //Sending 5 bytes

  if(bytesSent != outgoingPacketLength){
    std::cerr << "Number of bytes (" << bytesSent << ") sent not equal to five!\n";
    return false;
  }
  std::cout << "packet sent\n";
  return true;
}

bool HardwareCom::readController(){
    if(connection.available()>=incomingPacketLength){
        connection.read(incomingPacket, incomingPacketLength);
        connection.flushInput();
        std::cout << "First four bytes of packet received:   " << std::bitset<8>(incomingPacket[3]) << std::bitset<8>(incomingPacket[2]) << std::bitset<8>(incomingPacket[1]) << std::bitset<8>(incomingPacket[0]) << "\n";
        if(static_cast<uint8_t>(incomingPacket[0]^incomingPacket[1]^incomingPacket[2]^incomingPacket[3]^incomingPacket[4]^
                                incomingPacket[5]^incomingPacket[6]^incomingPacket[7]^incomingPacket[8]^incomingPacket[9]^
                                incomingPacket[10]^incomingPacket[11]) == static_cast<uint8_t>(incomingPacket[12])){
            std::cout << "PEC Correct\n";
            //Fill in channel array
            for(int i =0; i<incomingPacketLength-2; i+=2){
                channels[i/2] = static_cast<int>(incomingPacket[i] | (incomingPacket[i+1]<<8));
            }
            return true;
        } //check PEC byte
        return false;
    }
    return false;
}

int HardwareCom::getCh1() {
    return channels[0];
}

int HardwareCom::getCh2() {
    return channels[1];
}

int HardwareCom::getCh3() {
    return channels[2];
}

int HardwareCom::getCh4() {
    return channels[3];
}

int HardwareCom::getCh5() {
    return channels[4];
}

int HardwareCom::getCh6() {
    return channels[5];
}

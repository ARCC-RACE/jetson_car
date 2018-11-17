#include <racecar_hardware_interface/racecar_arduino_com.h>
#include <bitset>

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
  //Set vars to zero
  steeringPosition = 0;
  wheelVelocity = 0;

  //open serial port
  if(connection.isOpen()){
    std::cout << "Port " << port << " opened successfully!\n";
  } else {
    std::cerr << "Port " << port << " failed to open successfully!\n";
    exit(1);
  }
}

void HardwareCom::setController(int length, double* cmd){
  double steeringCmd = cmd[LEFT_STEERING_HINGE];
  double throttleCmd = cmd[LEFT_REAR_WHEEL];
  std::cout << "Steering cmd: " << steeringCmd << "\n";
  std::cout << "Throttle cmd: " << throttleCmd << "\n";
  auto steeringVal = static_cast<int>(map<double>(steeringCmd, -1, 1, -1000, 1000));
  auto throttleVal = static_cast<int>(map<double>(throttleCmd, -10, 10, -1000, 1000));
  constrain(steeringVal, -1000, 1000);

  //Safety precausion if throttle value goes out of bounds
  if(throttleVal > 1000 || throttleVal < -1000){
    std::cerr << "Throttle value out of bounds!\n";
    throttleVal = 0;
  }

  //Fill packetDown with four bytes with proper packet structure
  packetDown[3] = static_cast<uint8_t>(steeringVal >> 8);                 //Steering MSB
  packetDown[2] = static_cast<uint8_t>(steeringVal & 0b0000000011111111); //Steering LSB
  packetDown[1] = static_cast<uint8_t>(throttleVal >> 8);                 //Throttle  MSB
  packetDown[0] = static_cast<uint8_t>(throttleVal & 0b0000000011111111); //Throttle LSB

  packetDown[4] = static_cast<uint8_t >(packetDown[0]^packetDown[1]^packetDown[2]^packetDown[3]); //PEC by XORing all values

  std::cout << "packet down: " <<  std::bitset<8>(packetDown[3]) << std::bitset<8>(packetDown[2]) << std::bitset<8>(packetDown[1]) << std::bitset<8>(packetDown[0]) << "\n";

  size_t bytesSent = connection.write(packetDown, 5); //Sending 5 bytes

  if(bytesSent != 5){
    std::cerr << "Number of bytes (" << bytesSent << ") sent not equal to five!\n";
  }

  steeringPosition = steeringCmd; //Temp
  wheelVelocity = throttleCmd;    //Temp

  /*usleep(100); //sleep for 100 microseconds

  connection.read(packetUp, 4);
  std::cout << "packet up:   " << std::bitset<8>(packetUp[3]) << std::bitset<8>(packetUp[2]) << std::bitset<8>(packetUp[1]) << std::bitset<8>(packetUp[0]) << "\n";
  steeringPosition = static_cast<double>((packetUp[3] << 8) | packetUp[2]);
  wheelVelocity = static_cast<double>((packetUp[1] << 8) | packetUp[0]);

  //steeringPosition = map<double>(steeringPosition, 1100, 1900, -1000, 1000);
  wheelVelocity = map<double>(wheelVelocity, 1100, 1900, -1000, 1000); //Until encoder and IMU gets setup (this is not accurate at all)*/

}

double HardwareCom::readController(Joint joint){

  if(joint == LEFT_REAR_WHEEL || joint == RIGHT_REAR_WHEEL || joint == LEFT_FRONT_WHEEL || joint == RIGHT_FRONT_WHEEL){
    return(wheelVelocity);
  } else if(joint == LEFT_STEERING_HINGE || joint == RIGHT_STEERING_HINGE) {
    return(steeringPosition);
  } else {
    return 0;
  }

}

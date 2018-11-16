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


HardwareCom::HardwareCom(std::string port, int baud): connection(port, baud, serial::Timeout::simpleTimeout(10))
{
  //Set vars to zero
  steeringPosition = 0;
  wheelVelocity = 0;

  //open serial port
  if(connection.isOpen()){
    ROS_INFO("Port %s opened successfully!", port.c_str());
  } else {
    ROS_ERROR("Port %s failed to open successfully :(", port.c_str());
    exit(1);
  }
}

void HardwareCom::setController(int length, double* cmd){
  double steeringCmd = cmd[LEFT_STEERING_HINGE];
  double throttleCmd = cmd[LEFT_REAR_WHEEL];
  ROS_INFO_STREAM("Steering cmd: " << steeringCmd);
  ROS_INFO_STREAM("Throttle cmd: " << throttleCmd);
  int steeringVal =  map<double>(steeringCmd, -1, 1, -1000, 1000);
  int throttleVal =  map<double>(throttleCmd, -10, 10, -1000, 1000);
  constrain(steeringVal, -1000, 1000);

  //Safety precausion if throttle value goes out of bounds
  if(throttleVal > 1000 || throttleVal < -1000){
    ROS_ERROR("Throttle value out of bounds!");
    throttleVal = 0;
  }

  //Fill packetDown with four bytes with proper packet structure
  packetDown[3] = steeringVal >> 8;               //Steering MSB
  packetDown[2] = steeringVal & 0b0000000011111111; //Steering LSB
  packetDown[1] = throttleVal >> 8;               //Throttle  MSB
  packetDown[0] = throttleVal & 0b0000000011111111; //Throttle LSB

  size_t bytesSent = connection.write(packetDown, 4); //Sending 4 bytes

  if(bytesSent != 4){
    ROS_ERROR("Number of bytes sent not equal to four!");
  }

  steeringPosition = steeringCmd; //Temp
  wheelVelocity = throttleCmd; //Temp
  /*usleep(100); //sleep for 100 microseconds

  connection.read(packetUp, 4);
  steeringPosition = static_cast<double>((packetUp[3] << 8) | packetUp[2]);
  wheelVelocity = static_cast<double>((packetUp[1] << 8) | packetUp[1]);

  steeringPosition = map<double>(steeringPosition, 1100, 1900, -1000, 1000);
  wheelVelocity = map<double>(wheelVelocity, 1100, 1900, -1000, 1000); //Until encoder and IMU gets setup (this is not accurate at all)*/

}

double HardwareCom::readController(Joint joint){

  if(joint == LEFT_REAR_WHEEL || joint == RIGHT_REAR_WHEEL || joint == LEFT_FRONT_WHEEL || joint == RIGHT_FRONT_WHEEL){
    return(wheelVelocity);
  } else if(joint == LEFT_STEERING_HINGE || joint == LEFT_STEERING_HINGE) {
    return(steeringPosition);
  } else {
    return 0;
  }

}

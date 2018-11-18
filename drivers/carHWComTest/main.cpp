#include <iostream>
#include <unistd.h>
#include <racecar_hardware_interface/racecar_arduino_com.h>
#include <bitset>

int main() {
    HardwareCom arduinoConnection("/dev/ttyUSB0", 115200);
    double testArr[6] = {5, 5, 5, 5, 1, 0.5};
    while(true){
        for(double i = -1; i < 1; i+=0.001) {
            testArr[4] = 1;
            testArr[0] = 1;
            arduinoConnection.setController(6, testArr);
            std::cout << "steering pos: " << arduinoConnection.readController(static_cast<Joint>(4)) << "\n";
            std::cout << "throttle vel: " << arduinoConnection.readController(static_cast<Joint>(0)) << "\n\n";
            usleep(2000); //50Hz
        }
    }
    return 0;
}
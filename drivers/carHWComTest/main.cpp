#include <unistd.h>
#include <racecar_hardware_interface/racecar_arduino_com.h>

int main() {
    HardwareCom arduinoConnection("/dev/ttyACM0", 115200);
    while (true) {
        arduinoConnection.setController(0.5, 0.5);
        arduinoConnection.readController();

        std::cout << "CH1:" << arduinoConnection.getCh1() << " CH2:" << arduinoConnection.getCh2() << " CH3:"
        << arduinoConnection.getCh3() << " CH4:" << arduinoConnection.getCh4() << " CH5:"
        << arduinoConnection.getCh5() << " CH6:" << arduinoConnection.getCh6() << "\n";

        usleep(2000); //50Hz

        std::cout << "\n";
    }
    return 0;
}
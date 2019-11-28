#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "usb_hw_interface/racecar_arduino_com.h"
#include <unistd.h>
#include <cmath>


using std::placeholders::_1;
using namespace std::chrono_literals; //allows 50ms specification in timer call

// Helper functions
template <class T>
T map(T input, T inMin, T inMax, T outMin, T outMax){
    T output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    return output;
}

template <class T>
T constrain(T value, T min, T max){
    if(value > max){
        value = max;
    } else if(value < min){
        value = min;
    }
    return value;
}

// Hardware interface node
class HWInterface : public rclcpp::Node
{
public:
    HWInterface(): Node("hw_interface"), controller("/dev/ttyACM0", 115200)
    {
        cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("hw_cmd", 2, std::bind(&HWInterface::hw_cmd_callback, this, _1));
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 2);
        timer_ = this->create_wall_timer(10ms, std::bind(&HWInterface::timer_callback, this));

        steering = 0;
        throttle = 0;
    }

private:
    //update command variables
    void hw_cmd_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr cmd)
    {
        //steering and throttle should be -1.0 to 1.0
        RCLCPP_DEBUG(this->get_logger(), "Steering: %d Throttle: %d", cmd->drive.steering_angle, cmd->drive.speed);
        steering = cmd->drive.steering_angle;
        throttle = cmd->drive.speed;
    }

    //update joy and send steering/throttle to controller
    void timer_callback()
    {
        controller.readController();
        controller.setController(steering, throttle);

        auto joy_msg = sensor_msgs::msg::Joy();
        double ch1 = map(static_cast<double>(controller.getCh1()), 1000.0, 2000.0, -1.0, 1.0);
        joy_msg.axes.push_back(constrain(ch1, -1.0, 1.0));
        double ch2 = map(static_cast<double>(controller.getCh2()), 1000.0, 2000.0, -1.0, 1.0);
        joy_msg.axes.push_back(constrain(ch2, -1.0, 1.0));
        double ch5 = map(static_cast<double>(controller.getCh5()), 1000.0, 2000.0, -1.0, 1.0);
        joy_msg.axes.push_back(constrain(ch5, -1.0, 1.0));
        double ch6 = map(static_cast<double>(controller.getCh6()), 1000.0, 2000.0, -1.0, 1.0);
        joy_msg.axes.push_back(constrain(ch6, -1.0, 1.0));
        int ch3 = static_cast<int>(controller.getCh3());
        int ch4 = static_cast<int>(controller.getCh4());
        if(ch3<1500){
            ch3 = 0;
        } else {
            ch3 = 1;
        }
        if(ch4<1250){
            ch4 = 0;
        } else if(ch4<1750){
            ch4 = 1;
        } else {
            ch4 = 2;
        }
        joy_msg.buttons.push_back(ch3);
        joy_msg.buttons.push_back(ch4);
        joy_msg.header.stamp = this->now();
        joy_pub_->publish(joy_msg);
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    HardwareCom controller;
    volatile double steering;
    volatile double throttle;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HWInterface>());
    rclcpp::shutdown();
    return 0;
}
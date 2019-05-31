#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include "racecar_hardware_interface/racecar_arduino_com.h"

double steering = 0;
double throttle = 0;

std::string joints[6] = {"left_front_wheel_joint", "left_rear_wheel_joint", "right_front_wheel_joint",
  "right_rear_wheel_joint","left_steering_hinge_joint", "right_steering_hinge_joint"};

double cmd[6] = {0,0,0,0,0,0}; //stores values to update car

void updateThrottle(const std_msgs::Float64::ConstPtr& input){
  throttle = input->data;
}

void updateSteering(const std_msgs::Float64::ConstPtr& input){
  steering = input->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "temp_racecar_hardware_interface");
  ros::NodeHandle nh;

  HardwareCom connection("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A907NODW-if00-port0", 115200);

  ros::Subscriber left_rear_wheel_pub = nh.subscribe("/racecar/left_front_wheel_velocity_controller/command", 1, updateThrottle);
  ros::Subscriber left_steering_hinge_pub = nh.subscribe("/racecar/left_steering_hinge_position_controller/command", 1, updateSteering);

  ros::Publisher JointStatePub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

  ros::Rate rate(50); //50Hz

  while(ros::ok()){

    //Update command array
    //throttle should be between -10 and 10
    //steering should be between -1 and 1
    cmd[0] = throttle;
    cmd[1] = throttle;
    cmd[2] = throttle;
    cmd[3] = throttle;
    cmd[4] = steering;
    cmd[5] = steering;

    connection.setController(6, cmd); //Send values to arduino

    //Update joint states
    sensor_msgs::JointState joint_state_msg;

    for(int i = 0; i < 6; i++){
      joint_state_msg.name.push_back(joints[i]); //Treat as vectors
    }
    for(int i = 0; i < 4; i++){ //Increment through throttle joints
      joint_state_msg.position.push_back(0);
      joint_state_msg.velocity.push_back(throttle);
      joint_state_msg.effort.push_back(0);
    }
    for(int i = 4; i < 6; i++){ //Increment through steering joints
      joint_state_msg.position.push_back(steering);
      joint_state_msg.velocity.push_back(0);
      joint_state_msg.effort.push_back(0);
    }

    JointStatePub.publish(joint_state_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

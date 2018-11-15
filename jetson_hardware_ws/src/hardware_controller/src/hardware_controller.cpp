#include <ros/ros.h>


int main(int argc, char** argv){

   ros::init(argc, argv, "hardwareController");
   ros::NodeHandle nh;
   ROS_INFO("Hardware Controller Node Initialized");
   
   while(ros::ok()){;

   }

   return 0;
}

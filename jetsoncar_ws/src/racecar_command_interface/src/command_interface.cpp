#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <racecar_command_interface/RacecarCommandInterfaceConfig.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

//declare ROS publsihers
ros::Publisher safety_pub;
ros::Publisher autonomous_pub;
ros::Publisher cruise_vel_pub;
ros::Publisher race_pub;

void safetyPing(const ros::TimerEvent&){
  std_msgs::Empty msg;
  safety_pub.publish(msg);
}

void callback(racecar_command_interface::RacecarCommandInterfaceConfig &config, uint32_t level) {
  ROS_DEBUG("Reconfigure Request: %s %f %s ", config.cruise_velocity,
                                              config.autonomous_mode?"True":"False",
                                              config.data_collection_mode?"True":"False",
                                              config.race_mode?"True":"False",
                                              config.STOP?"True":"False");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "racecar_command_interface");
  ros::NodeHandle nh;

  //Setup dynamic reconfigure
  dynamic_reconfigure::Server<racecar_command_interface::RacecarCommandInterfaceConfig> server;
  dynamic_reconfigure::Server<racecar_command_interface::RacecarCommandInterfaceConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  //+20Hz publisher to act as safety mechanism for controller (dead mans switch)
  safety_pub = nh.advertise<std_msgs::Empty>("/racecar/safety", 2);
  ros::Timer safetyTimer = nh.createTimer(ros::Duration(0.05), safetyPing); //callback to send safety ping (1/Hz)

  autonomous_pub = nh.advertise<std_msgs::Bool>("/racecar/autonomous_mode", 2);
  cruise_vel_pub = nh.advertise<std_msgs::Float64>("/racecar/cruise_velocity", 2);
  race_pub = nh.advertise<std_msgs::Bool>("/racecar/race_mode", 2);

  ROS_INFO("Racecar Command Interface Launched");

  while(ros::ok()){
    ros::spinOnce(); //Check CB loop
  }

  return 0;
}

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
ros::Publisher data_recorder_pub;

bool isStopped = false; //Var to stop publishing safety msg to stop contorller

void safetyPing(const ros::TimerEvent&){
  if(!isStopped){
    std_msgs::Empty msg;
    safety_pub.publish(msg);
  }
}

void callback(racecar_command_interface::RacecarCommandInterfaceConfig &config, uint32_t level) {
  ROS_DEBUG("Reconfigure Request: %f %s %s %s %s", config.cruise_velocity,
                                              config.autonomous_mode?"True":"False",
                                              config.data_collection_mode?"True":"False",
                                              config.race_mode?"True":"False",
                                              config.STOP?"True":"False");

  std_msgs::Float64 cruise_vel_msg;
  std_msgs::Bool autonomous_mode_msg;
  std_msgs::Bool race_mode_msg;
  std_msgs::Bool data_recorder_msg;

  cruise_vel_msg.data = config.cruise_velocity;
  autonomous_mode_msg.data = config.autonomous_mode;
  race_mode_msg.data = config.race_mode;
  data_recorder_msg.data = config.data_collection_mode;

  cruise_vel_pub.publish(cruise_vel_msg);
  autonomous_pub.publish(autonomous_mode_msg);
  race_pub.publish(race_mode_msg);
  data_recorder_pub.publish(data_recorder_msg);

  isStopped = static_cast<bool>(config.STOP);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "command_interface");
  ros::NodeHandle nh;

  //+20Hz publisher to act as safety mechanism for controller (dead mans switch)
  safety_pub = nh.advertise<std_msgs::Empty>("/racecar/safety", 2);
  ros::Timer safetyTimer = nh.createTimer(ros::Duration(0.05), safetyPing); //callback to send safety ping (1/Hz)

  autonomous_pub = nh.advertise<std_msgs::Bool>("/racecar/autonomous_mode", 2);
  cruise_vel_pub = nh.advertise<std_msgs::Float64>("/racecar/cruise_velocity", 2);
  race_pub = nh.advertise<std_msgs::Bool>("/racecar/race_mode", 2);
  data_recorder_pub = nh.advertise<std_msgs::Bool>("/racecar/record_data", 2);

  //Setup dynamic reconfigure
  dynamic_reconfigure::Server<racecar_command_interface::RacecarCommandInterfaceConfig> server;
  dynamic_reconfigure::Server<racecar_command_interface::RacecarCommandInterfaceConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Racecar Command Interface Launched");

  ros::spin(); //Enter into callback loop

  return 0;
}

#ifndef RACECAR_HARDWARE_INTERFACE_H
#define RACECAR_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <ros/ros.h>
#include <memory> //For shared pointer wrapper

class Racecar : public hardware_interface::RobotHW
{
  hardware_interface::JointStateInterface jnt_state_interface;
  //hardware_interface::PositionJointInterface jnt_pos_interface;
  //hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;

  //Number of joints being contorlled = 6
  double cmd[6];
  double pos[6];
  double vel[6];
  double eff[6];

  ros::NodeHandle& _nh1; //Node handle for dealing with the controller_manager
  ros::NodeHandle& _nh2; //Node handle for dealing with hardware_interface timers
  std::shared_ptr<controller_manager::ControllerManager> _cm;

  ros::Duration loop_time; // 1/50  ros::Duration elapsed_time(0.02);
  ros::Timer timer_loop;

public:
  Racecar(ros::NodeHandle& nh1, ros::NodeHandle& nh2);
  void read(); //update the JointSate value
  void write(); //update the motor/servo to the joints setpoint
  void update(const ros::TimerEvent& e);
};

#endif

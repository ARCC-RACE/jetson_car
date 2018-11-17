// https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot
// http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface
#include "racecar_hardware_interface/racecar_hardware_interface.h"

Racecar::Racecar(ros::NodeHandle& nh1, ros::NodeHandle& nh2): nh1_(nh1), nh2_(nh2), loop_time(0.02), hwController("/dev/ttyACM0", 115200)
 {
   controller_manager::ControllerManager *cm = new controller_manager::ControllerManager(this, nh1_); //start the controller manager
   cm_.reset(cm); //Transfer pointer into a shared_ptr wrapper for protection

   timer_loop = nh2_.createTimer(loop_time, &Racecar::update, this);

   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_left_rear_wheel("left_rear_wheel_joint", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_left_rear_wheel);

   hardware_interface::JointStateHandle state_handle_right_rear_wheel("right_rear_wheel_joint", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_right_rear_wheel);

   hardware_interface::JointStateHandle state_handle_left_front_wheel("left_front_wheel_joint", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_left_front_wheel);

   hardware_interface::JointStateHandle state_handle_right_front_wheel("right_front_wheel_joint", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_right_front_wheel);

   hardware_interface::JointStateHandle state_handle_left_steering("left_steering_hinge_joint", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(state_handle_left_steering);

   hardware_interface::JointStateHandle state_handle_right_steering("right_steering_hinge_joint", &pos[5], &vel[5], &eff[5]);
   jnt_state_interface.registerHandle(state_handle_right_steering);

   registerInterface(&jnt_state_interface);

   // connect and register the joint effort interface
   hardware_interface::JointHandle vel_handle_left_rear_wheel(jnt_state_interface.getHandle("left_rear_wheel_joint"), &cmd[0]);
   jnt_eff_interface.registerHandle(vel_handle_left_rear_wheel);

   hardware_interface::JointHandle vel_handle_right_rear_wheel(jnt_state_interface.getHandle("right_rear_wheel_joint"), &cmd[1]);
   jnt_eff_interface.registerHandle(vel_handle_right_rear_wheel);

   hardware_interface::JointHandle vel_handle_left_front_wheel(jnt_state_interface.getHandle("left_front_wheel_joint"), &cmd[2]);
   jnt_eff_interface.registerHandle(vel_handle_left_front_wheel);

   hardware_interface::JointHandle vel_handle_right_front_wheel(jnt_state_interface.getHandle("right_front_wheel_joint"), &cmd[3]);
   jnt_eff_interface.registerHandle(vel_handle_right_front_wheel);

   hardware_interface::JointHandle pos_handle_left_steering(jnt_state_interface.getHandle("left_steering_hinge_joint"), &cmd[4]);
   jnt_eff_interface.registerHandle(pos_handle_left_steering);

   hardware_interface::JointHandle pos_handle_right_steering(jnt_state_interface.getHandle("right_steering_hinge_joint"), &cmd[5]);
   jnt_eff_interface.registerHandle(pos_handle_right_steering);

   registerInterface(&jnt_eff_interface);
  }

//Set the physical values for the joint
//number of joints = 6
void Racecar::write(){
  hwController.setController(6, cmd);
  /*for(int i=0; i<6; i++){ //Iterate through all 6 joints
    ROS_DEBUG_STREAM("Joint data: " << i << " " << cmd[i]);
  }*/
}

//Read the joint sensors in order to publish joint state
void Racecar::read(){
  for(int i=0; i<4; i++){ //Iterate through 4 effort velocity joints
    vel[i] = hwController.readController(static_cast<Joint>(i))
    ROS_DEBUG_STREAM("Wheel velocity: " << vel[i]);
  }
  for(int i=4; i<6; i++){ //Iterate through 2 effort position joints
    pos[i] = hwController.readController(static_cast<Joint>(i));
    ROS_DEBUG_STREAM("Steering position: " << pos[i]);
  }
}

//Update the joints
void Racecar::update(const ros::TimerEvent& e){
  ros::Duration elapsed_time = ros::Duration(e.current_real - e.last_real);
  read();
  cm_->update(ros::Time::now(), elapsed_time);
  write();
}

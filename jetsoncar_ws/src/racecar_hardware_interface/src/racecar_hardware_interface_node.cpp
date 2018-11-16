#include "racecar_hardware_interface/racecar_hardware_interface.h"
#include <ros/callback_queue.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "racecar_hardware_interface");

  ros::NodeHandle nh1; //Node handle for dealing with the controller_manager
  ros::NodeHandle nh2; //Node handle for dealing with hardware_interface CB timers

  /*
  Two seperate callback queues are required to properly handle the
  services of the control Controller_Manager and the callbacks of the
  ros::timer that updates the hardware_interface values.

  See https://answers.ros.org/question/119316/using-controller-manager-and-getting-it-to-work/
  */

  ros::CallbackQueue nh1_callback_queue;
  ros::CallbackQueue nh2_callback_queue;

  nh1.setCallbackQueue(&nh1_callback_queue);
  nh2.setCallbackQueue(&nh2_callback_queue);

  ros::AsyncSpinner spinner1(0, &nh1_callback_queue);
  spinner1.start();

  ros::AsyncSpinner spinner2(0, &nh2_callback_queue);
  spinner2.start();

  Racecar racecar(nh1, nh2);
  ros::waitForShutdown();

  return 0;
}

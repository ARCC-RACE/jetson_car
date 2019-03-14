Setup Instrucitons:

Install Open AI Gym + Gazebo extension:
Does this need to be python 3?
* Use setup.sh for Ubuntu 16.04 if desired
* Use docker to run Gazebo 8.1.1 or greater with ROS kinetic?
* Make sure to do the install for python 3.5.2 or greater (pip3)
* Run `echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc` if needed to deal with lingering processes
* https://github.com/openai/gym#installation
* https://github.com/erlerobot/gym-gazebo/blob/master/README.md

Read the getting started docs:
* https://gym.openai.com/docs/

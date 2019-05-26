Setup Instructions:

IS PYTHON 3 Needed? If python 2 is usable then it should be preferred.

Install Open AI Gym + Gazebo extension:
Does this need to be python 3? No
* Use setup.sh for Ubuntu 16.04 if desired
* Use docker to run Gazebo 8.1.1 or greater with ROS kinetic?
* Make sure to do the install for python 3.5.2 or greater (pip3)
* Run `echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc` if needed to deal with lingering processes
* https://github.com/openai/gym#installation
* https://github.com/erlerobot/gym-gazebo/blob/master/README.md

Read the getting started docs:
* https://gym.openai.com/docs/


You may need to install these packages as well if you receive warnings when running the learn script:
* `sudo pip install -U pyopenssl`
* `sudo pip install --upgrade cryptography`

Reminder that you will need to manually close the program due to how Open AI Gym scripts are constructed
* `ps faux | grep python`
* `kill -9 id__your_script_process`
ex
   * `michael  17175 10.4 0.8 3970596 291424 pts/21 Tl 10:49 0:02 | | \_ python ./hal_learn.py`
   * `kill -9 17175`

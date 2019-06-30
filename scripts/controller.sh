# Set up the correct ROS_MASTER_URI and hostname
if ! grep "export ROS_HOSTNAME=$(hostname).local" ~/.bashrc ; then
  echo -e "\n#################### Added by controller.sh for autonomous racecar ####################" >> ~/.bashrc
  echo "# Comment out the lines below when using ROS for other applications" >> ~/.bashrc
  echo "export ROS_HOSTNAME=$(hostname).local" >> ~/.bashrc
  echo -e "What is the hostname of the racecar you are using? (do not include suffix (i.e. *.local) and make sure that pinging racecar_hostname.local from controller works)\n Run \`hostname\` in a terminal on the jetson to see what the hostname is"
  read racecar_hostname
  echo "ROS_MASTER_URI=http://$(racecar_hostname).local:11311"
  echo -e "export ROS_MASTER_URI=http://$(racecar_hostname).local:11311\n" >> ~/.bashrc
  echo "Adding ~/Desktop/jetson_car/jetsoncar_ws/devel/setup.bash in ~/.bashrc -> change if needed!"
  echo "source ~/Desktop/jetson_car/jetsoncar_ws/devel/setup.bash" >> ~/.bashrc
fi

source ~/.bashrc

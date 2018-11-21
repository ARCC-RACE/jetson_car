if ! grep "export ROS_HOSTNAME=racecar" ~/.bashrc ; then
   echo -e "\nexport ROS_HOSTNAME=racecar" >> ~/.bashrc
   echo "export ROS_MASTER_URI=http://racecar:11311" >> ~/.bashrc
fi

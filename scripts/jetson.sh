if ! grep "export ROS_HOSTNAME=racecar" ~/.bashrc ; then
   echo -e "\nexport ROS_HOSTNAME=racecar" >> ~/.bashrc
   echo "export ROS_MASTER_URI=http://racecar:11311" >> ~/.bashrc
fi

if ! grep "192.168.1.100 racecar" /etc/hosts ; then
   echo -e "\n192.168.1.100 racecar" >> /etc/hosts
fi

#!/bin/bash

if ! grep "192.168.1.100 master" /etc/hosts ; then
   sudo -- sh -c "echo 192.168.1.100   master >> /etc/hosts"
fi

if ! grep "192.168.1.101 control" /etc/hosts ; then
   sudo -- sh -c "echo 192.168.1.101   control >> /etc/hosts"
fi


if [ $(hostname -I) == "192.168.1.100" ]; then
   echo "You are on the jetson with the IP of $(hostname -I)"
   if ! grep "export ROS_HOSTNAME=master" ~/.bashrc ; then
      echo -e "\nexport ROS_HOSTNAME=master" >> ~/.bashrc
      echo "export ROS_MASTER_URI=http://master:11311" >> ~/.bashrc
   fi
   echo -e "\n\nYou are now configured as master!"

elif [ $(hostname -I) == "192.168.1.101" ]; then
   echo "You are on the control computer with the IP of $(hostname -I)"
   if ! grep "export ROS_HOSTNAME=master" ~/.bashrc ; then
      echo -e "\nexport ROS_HOSTNAME=control" >> ~/.bashrc
      echo "export ROS_MASTER_URI=http://master:11311" >> ~/.bashrc
   fi
   echo -e "\n\nYou are now configured as control!"

else
   echo "Your IP ($(hostname -I)) does not match a designated static IP!"
   echo "Please set up a proper static IP in the network connections app (see github README for more details)"

fi

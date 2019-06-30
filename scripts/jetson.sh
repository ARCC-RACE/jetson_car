# Set up the correct ROS_MASTER_URI and hostname
if ! grep "export ROS_HOSTNAME=$(hostname).local" ~/.bashrc ; then
  echo -e "\n#################### Added by jetson.sh for autonomous racecar ####################" >> ~/.bashrc
  echo "# Comment out the lines below when using for other applications" >> ~/.bashrc
  echo "export ROS_HOSTNAME=$(hostname).local" >> ~/.bashrc
  echo "export ROS_MASTER_URI=http://$(hostname).local:11311" >> ~/.bashrc
fi

# Make sure all programs have access to the I2C bus devices (primarily for IMU)
if ! grep "sudo chmod a+rw /dev/i2c-1" ~/.bashrc ; then
   echo "\nsudo chmod a+rw /dev/i2c-1" >> ~/.bashrc
fi

# Set the wifi power_save mode off and clock up the nvidia jetson
if ! grep "sudo iw wlan0 set power_save off" ~/.bashrc ; then
   echo "\nsudo iw wlan0 set power_save off" >> ~/.bashrc
fi
if ! grep "sudo ~/jetson_clocks.sh" ~/.bashrc ; then
   echo -e "sudo ~/jetson_clocks.sh\n" >> ~/.bashrc
fi

source ~/.bashrc

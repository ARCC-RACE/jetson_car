# Set up the correct ROS_MASTER_URI and hostname
if ! grep "export ROS_HOSTNAME=$(hostname).local" ~/.bashrc ; then
  echo -e "\n#################### Added by jetson.sh for autonomous racecar ####################" >> ~/.bashrc
  echo "# Comment out the lines below when using for other applications" >> ~/.bashrc
  echo "export ROS_HOSTNAME=$(hostname).local" >> ~/.bashrc
  echo "export ROS_MASTER_URI=http://$(hostname).local:11311" >> ~/.bashrc
  echo "Adding ~/Desktop/jetson_car/jetsoncar_ws/devel/setup.bash in ~/.bashrc -> change if needed!"
  echo "source ~/Desktop/jetson_car/jetsoncar_ws/devel/setup.bash" >> ~/.bashrc
fi

# Add usergroup for program acces to USB
sudo usermod -a -G dialout $USER

# Make sure all programs have access to the I2C bus devices (primarily for IMU)
if ! find | grep "/etc/udev/rules.d/50-i2c.rules" ; then
  echo "writing new udev rule /etc/udev/rules.d/50-i2c.rules..."
  sudo su
  touch "/etc/udev/rules.d/50-i2c.rules"
  echo -e "ACTION==\"add\", KERNEL==\"i2c-[0-1]*\", MODE=\"0666\"" >> /etc/udev/rules.d/50-i2c.rules
  exit
fi

# Set the wifi power_save mode off and clock up the nvidia jetson
if ! grep "sudo iw wlan0 set power_save off" /etc/rc.local ; then
   echo "sudo iw wlan0 set power_save off" >> /etc/rc.local
fi
if ! grep "sudo ~/jetson_clocks.sh" /etc/rc.local ; then
   echo -e "sudo ~/jetson_clocks.sh\n" >> /etc/rc.local
fi

echo "Reboot your jetson"
source ~/.bashrc

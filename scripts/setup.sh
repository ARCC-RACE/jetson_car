#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

<<<<<<< HEAD
sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/buildPatchedKernel.sh"
echo "RESTART SYSTEM"
#sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/installLibrealsense.sh"
#sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/scripts/patchKernel.sh"
#sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/scripts/configureKernel.sh"
#sudo "${MY_PATH}/jetsonhacks/installROSTX2/installROS.sh -p ros-kinetic-desktop"
=======
#Requires update in order to properly source enviroment vars
sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/installLibrealsense.sh" #RESTART Needed after install! Check if it can be held off until full installation finishes

sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/scripts/patchKernel.sh"
sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/scripts/configureKernel.sh"
sudo "${MY_PATH}/jetsonhacks/installROSTX2/installROS.sh -p ros-kinetic-desktop"
>>>>>>> 998745985b39cedbeb8b8c5d2c30c26bc11235c2
#sudo "${MY_PATH}/jetsonhacks/installROSTX2/setupCatkinWorkspace.sh"
#sudo "${MY_PATH}/jetsonhacks/installRealSense2ROSTX/installRealSenseROS.sh jetsoncar_ws"


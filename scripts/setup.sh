#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

#Requires update in order to properly source enviroment vars
sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/buildPatchedKernel.sh"
#RESTART Needed after install! Check if it can be held off until full installation finishes
echo "RESTART SYSTEM"

#sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/installLibrealsense.sh"
#sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/scripts/patchKernel.sh" #DON'T USE
#sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/scripts/configureKernel.sh" #DON'T USE
#sudo "${MY_PATH}/jetsonhacks/installROSTX2/installROS.sh -p ros-kinetic-desktop"
#sudo "${MY_PATH}/jetsonhacks/installROSTX2/setupCatkinWorkspace.sh"
#sudo "${MY_PATH}/jetsonhacks/installRealSense2ROSTX/installRealSenseROS.sh jetsoncar_ws"

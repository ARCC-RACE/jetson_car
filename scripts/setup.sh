#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

#Requires update in order to properly source enviroment vars
sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/installLibrealsense.sh" #RESTART Needed after install! Check if it can be held off until full installation finishes

sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/scripts/patchKernel.sh"
sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/scripts/configureKernel.sh"
sudo "${MY_PATH}/jetsonhacks/installROSTX2/installROS.sh -p ros-kinetic-desktop"
#sudo "${MY_PATH}/jetsonhacks/installROSTX2/setupCatkinWorkspace.sh"
sudo "${MY_PATH}/jetsonhacks/installRealSense2ROSTX/installRealSenseROS.sh ${MY_PATH}/../jetsoncar_ws"


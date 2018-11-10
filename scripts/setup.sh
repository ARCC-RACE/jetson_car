#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/buildPatchedKernel.sh"
echo "RESTART SYSTEM"
#sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/installLibrealsense.sh"
#sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/scripts/patchKernel.sh"
#sudo "${MY_PATH}/jetsonhacks/buildLibrealsense2TX/scripts/configureKernel.sh"
#sudo "${MY_PATH}/jetsonhacks/installROSTX2/installROS.sh -p ros-kinetic-desktop"
#sudo "${MY_PATH}/jetsonhacks/installROSTX2/setupCatkinWorkspace.sh"
#sudo "${MY_PATH}/jetsonhacks/installRealSense2ROSTX/installRealSenseROS.sh jetsoncar_ws"


# jetson_car
Repository for the development of the JHS Autonomous Race Car Club's ROS  Jetson Car (TX2) with RealSense D400 series 3D camera

Check out system [flowcharts](https://drive.google.com/open?id=1oScZmCizjCHx7lIW-BQ1HAR4U3CumyJK)

Master   
[![Build Status](https://travis-ci.com/JHS-ARCC-Club/jetson_car.svg?branch=master)](https://travis-ci.com/JHS-ARCC-Club/jetson_car)
[![CodeFactor](https://www.codefactor.io/repository/github/jhs-arcc-club/jetson_car/badge/master)](https://www.codefactor.io/repository/github/jhs-arcc-club/jetson_car/overview/master)

Development   
[![Build Status](https://travis-ci.com/JHS-ARCC-Club/jetson_car.svg?branch=development)](https://travis-ci.com/JHS-ARCC-Club/jetson_car)
[![CodeFactor](https://www.codefactor.io/repository/github/jhs-arcc-club/jetson_car/badge/development)](https://www.codefactor.io/repository/github/jhs-arcc-club/jetson_car/overview/development)

## Getting Started
- Make sure you have installed git lfs on the jetson, controller, and development computer
   - Git LFS will allow you to easily keep ML models up to date across computers with git version control
   - https://git-lfs.github.com/
   - [Install of git lfs](https://github.com/git-lfs/git-lfs/wiki/Installation)
      - This will need to be installed by source on the [Jetson](https://github.com/Netzeband/JetsonTX1_im2txt/wiki/JetsonBasicSetupGit) (DO NOT follow the instructions exactly. See below)
         - Get latest version of [Go for Linux ARMv8](https://golang.org/dl/) and run the extract command
         - Make sure that the $GOPATH is set to home/$USER/gocode and that Go is version 1.8 or higher `go version`
         - `go get github.com/git-lfs/git-lfs`
         - `sudo cp ~/gocode/bin/git-lfs /usr/local/bin`
         - `git lfs install`
- Connect the nvidia jetson to the same network as the controller and make sure that you have an internet connection
- Before setup make sure to `sudo apt-get update` and `sudo apt-get upgrade`
- Clone/download the repository on both the jetson and the controller
- See the `Nvidia Jetson Setup` section
- If running on a desktop computer / controller without realsense packages installed run `rosdep install --from-paths src --ignore-src -r -y` and `catkin_make -DCATKIN_BLACKLIST_PACKAGES="realsense2_camera"` in workspace
   - If you want to run the realsense2 ROS node on a desktop / controller see `Librealsense Setup (D415/D435)` section and use the realsense2_camera pkg in the 	`drivers` folder by copying it into a local catkin_ws (do not push this change as it will brake the jetson side)
- On the jetson run `rosdep install --from-paths src --ignore-src -r -y` and `catkin_make` in the jetsoncar_ws
- Run the controller.sh or jetson.sh scripts to get the devices setup and communicating with each other OR run `export ROS_MASTER_URI=http://tegra-ubuntu.local:11311` and `export ROS_HOSTNAME=hostname_of_machine.local`
   - It is highly recommended that you run the controller.sh and jetson.sh scripts on their respective devices
- See `DualShock 4 Controller Setup` section for setting up the controller with the correct platform

#### Important Notes
- When running the jetson car with WiFi make sure that WiFi power saving mode is off. `sudo iw dev wlan0 set power_save off` and check with `sudo iw dev wlan0 get power_save`. This needs to be done each time the WiFi reconnects to a new network.

### Simulation Setup
- Go through Librealsense Setup (demo/viewing tools are not required)
- `cd ~/Desktop/jetson_car/jetsoncar_ws`
- `catkin_make` (even if this fails)
- `soruce devel/setup.bash`
- `rosdep install racecar_description`
- Install [Tensor Flow](https://www.tensorflow.org/install/) (download GPU version if you have one)
     - `sudo pip install tensorflow-gpu`
     - If running python program fails with error `ImportError: libcudnn.so.7: cannot open shared object file: No such file or directory` see [this](https://stackoverflow.com/questions/41991101/importerror-libcudnn-when-running-a-tensorflow-program but make sure to download proper version of cuDNN) [version 9.0](https://developer.nvidia.com/rdp/cudnn-download)
- Install Keras `sudo pip install keras`
- `sudo pip install h5py`

### Librealsense Setup (D415/D435)
- Install the intel realsense 2 SDK (needed to catkin_make the catkin workspace if using the simulation of the physical sensor)
- https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
   - `sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE`
   - `sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u`
   - `sudo apt-get update`
   - `sudo apt-get install librealsense2 librealsense2-dev librealsense2-dbg`
   - `sudo apt-get install librealsense2-dkms`  For demo
   - `sudo apt-get install librealsense2-utils` Tools for viewing physical camera data (See `/usr/bin/realsense-viewer`)
- When ready plug in the realsense camera and test it using the `/usr/bin/realsense-viewer`
   - If needed update the firmware using the instruction found [here](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Linux-RealSense-D400-DFU-Guide.pdf)


### Nvidia Jetson Setup
- Follow Nvidia Jetson setup and run as user `nvidia` (password nvidia)
- Check to make sure you are running L4T version 28.2.1 for use with convenience scripts
   - If you version is off download jetpack [here](https://developer.nvidia.com/embedded/downloads#?search=jetpack%203.3)
   - Follow [this tutorial on reflashing the Jetson TX2](https://www.youtube.com/watch?v=D7lkth34rgM)
- make sure you have cloned https://github.com/JHS-ARCC-Club/jetson_car.git to Desktop
- Run the following scripts in this order (in scripts/jetsonhacks): installLibrealsense.sh, buildPatchedKernal.sh, installROS.sh
- [Install USB driver for some arduino nano](https://devtalk.nvidia.com/default/topic/1032862/jetson-tx2/a-guide-to-solve-usb-serial-driver-problems-on-tx2/)
- Run the `/scripts/jetson.sh` script to setup network
- Setup pip and download keras and tensorflow for python
   - `sudo apt-get install -y python-pip`
   - `pip install keras`
   - `sudo pip install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp33 tensorflow-gpu`
- The following are done in the `jetson.sh` script but provided here for reference
   - `sudo usermod -a -G dialout $USER` to give proper permissions to the USB peripherals
   - Add a udev rule for i2c-1 so that the IMU interface program can access it
      - Go to `/etc/udev/rules.d`
      - Add a file called `50-i2c.rules`
      - Add the line `ACTION=="add", KERNEL=="i2c-[0-1]*", MODE="0666"` to the `50-i2c.rules` file
      - See [this](https://forum.up-community.org/discussion/2141/tutorial-gpio-i2c-spi-access-without-root-permissions) for additional information

### [RTAB Mapping setup](https://github.com/introlab/rtabmap_ros#installation)
- `sudo apt-get install ros-kinetic-rtabmap-ros ros-kinetic-robot-localization ros-kinetic-pointcloud-to-laserscan ros-kinetic-depthimage-to-laserscan`

### DualShock 4 Controller Setup (Ubuntnu 16.04)
- Connect to the controller by pressing and holding the `playstation button` and the `share` button until the light begins flashing white
- Go into Bluetooth settings and add the controller named `wireless controller`
- Test the connection by making sure the `/dev/input/js0` file exits
- Run the `joy_node` ROS node and echo the `/joy` topic to confirm data is being sent properly from the controller to the computer


### DualShock 4 Controller Setup (Ubuntu MATE RPI)
- Install `bluez` driver `sudo apt-get install bluez`
- https://github.com/macunixs/dualshock4-pi/blob/master/README.md
- Follow the steps for Ubutnu 16.04 above

## Running the tests

See the travis.yaml for running install/make tests and verifying a proper build.

## Built With

* [ROS](https://www.ros.org/) - Modern robot framework for running software
* [librealsense](https://github.com/IntelRealSense/librealsense) - Intel Realsense library for D415/D435
* [Git LFS](https://git-lfs.github.com/) - Manage and store trained AI models
* [Travis CI](https://travis-ci.org/) - Perform build tests

## Authors

* **Michael Equi** - *Initial work*

See also the list of [contributors](https://github.com/JHS-ARCC-Club/jetson_car/graphs/contributors) who participated in this project.

## Acknowledgments

* README Tempalte source https://gist.github.com/PurpleBooth/109311bb0361f32d87a2

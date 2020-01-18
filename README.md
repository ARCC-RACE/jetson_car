# jetson_car
Repository for the development of the JHS Autonomous Race Car Club's ROS  Jetson Car (TX2) with RealSense D400 series 3D camera

Check out system [flowcharts](https://drive.google.com/open?id=1oScZmCizjCHx7lIW-BQ1HAR4U3CumyJK)

Master   
[![Build Status](https://travis-ci.com/ARCC-RACE/jetson_car.svg?branch=master)](https://travis-ci.com/ARCC-RACE/jetson_car)
[![CodeFactor](https://www.codefactor.io/repository/github/ARCC-RACE/jetson_car/badge/master)](https://www.codefactor.io/repository/github/jhs-arcc-club/jetson_car/overview/master)

Development   
[![Build Status](https://travis-ci.com/ARCC-RACE/jetson_car.svg?branch=development)](https://travis-ci.com/ARCC-RACE/jetson_car)
[![CodeFactor](https://www.codefactor.io/repository/github/arcc-race/jetson_car/badge/development)](https://www.codefactor.io/repository/github/arcc-race/jetson_car/overview/development)

## Getting Started
- Make sure that you have [installed ROS2](https://index.ros.org/doc/ros2/Installation/Dashing/)
- Make sure you have installed git lfs on the jetson, controller, and development computer
- Connect the nvidia jetson to the same network as the controller and make sure that you have an internet connection
- Before setup make sure to `sudo apt-get update` and `sudo apt-get upgrade`
- Clone/download the repository on both the jetson and the controller
- See the `Nvidia Jetson Setup` section
- ~Install EmPy `pip3 install EmPy` or `python3 -m pip install EmPy`
- Install Lark `pip3 install lark-parser` or `python3 -m pip install lark-parser`~
- `sudo apt-get install python-rosdep`
- Install non ROS dependencies for intel realsense `sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev;sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev`
- Run `rosdep install --from-paths src --ignore-src -r -y` and `colcon build --symlink-install`
- On the jetson run `rosdep install --from-paths src --ignore-src -r -y` and `colcon build --symlink-install` in the jetsoncar_ws
   - If you have issues building the realsense node see [this](https://github.com/intel/ros2_intel_realsense/issues/93)
- Run the controller.sh or jetson.sh scripts to get the devices setup and communicating with each other OR run `export ROS_MASTER_URI=http://tegra-ubuntu.local:11311` and `export ROS_HOSTNAME=hostname_of_machine.local`
   - It is highly recommended that you run the controller.sh and jetson.sh scripts on their respective devices

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
   - `sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u`
   - `sudo apt-get update`
   - `sudo apt-get install librealsense2 librealsense2-dev librealsense2-dbg`
   - `sudo apt-get install librealsense2-dkms`  For demo
   - `sudo apt-get install librealsense2-utils` Tools for viewing physical camera data (See `/usr/bin/realsense-viewer`)
- When ready plug in the realsense camera and test it using the `/usr/bin/realsense-viewer`
   - If needed update the firmware using the instruction found [here](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Linux-RealSense-D400-DFU-Guide.pdf)


### Nvidia Jetson Setup
- Follow Nvidia Jetson setup and run as user `jetsoncar` (password jetsoncar)
- Check to make sure you are running L4T version 32.2.1 for use with convenience scripts
   - If you version is off download jetpack [here](https://developer.nvidia.com/embedded/downloads#?search=jetpack%203.3)
   - Follow [this tutorial on reflashing the Jetson TX2](https://www.youtube.com/watch?v=D7lkth34rgM)
- make sure you have cloned https://github.com/JHS-ARCC-Club/jetson_car.git to home directory and init/updated submodules
- Setup pip and download keras and tensorflow for python3
   - [Install python3 tensorflow](https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html)
   - `sudo apt install gfortran python3-scipy python3-keras`
   - If you encounter errors loading the models try uninstalling keras and installing the latest version
      - `sudo python3 -m pip uninstall keras`
      - `sudo python3 -m pip install keras`
- `sudo apt-get install python3-libnvinfer-dev uff-converter-tf`
- Run the `/scripts/jetson.sh` script to setup network
- The following are done in the `jetson.sh` script but provided here for reference
   - `sudo chmod a+rw /dev/ttyACM0` to give proper permissions to the USB peripherals
   - Add a udev rule for i2c-1 so that the IMU interface program can access it
      - Go to `/etc/udev/rules.d`
      - Add a file called `50-i2c.rules`
      - Add the line `ACTION=="add", KERNEL=="i2c-[0-1]*", MODE="0666"` to the `50-i2c.rules` file
      - See [this](https://forum.up-community.org/discussion/2141/tutorial-gpio-i2c-spi-access-without-root-permissions) for additional information

### Jetson ROS install

1. Install ROS2 following [this link](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/). Make sure you also echo your source command into the bashrc and install autocomplete. You should be able to install ROS through debian packages. No need to build from source.

2. Test the ros2 build. Run `ros2 run demo_nodes_cpp talker` in one terminal and `ros2 run demo_nodes_py listener` in another. You should see the `talker` saying that it’s Publishing messages and the `listener` saying I heard those messages. 

3. Install ROS2 Joy node.`sudo apt-get install ros-dashing-joy`.

4. Test ROS2 Joy node by running `ros2 run joy joy-node`, plugging in a controller, then opening another terminal and runnign `ros2 topic echo /joy`, and a stream should of joystick commands be visible.

## Running the tests

See the travis.yaml for running install/make tests and verifying a proper build.

## Built With

* [ROS2](https://index.ros.org/doc/ros2/) - Modern robot framework for running software
* [librealsense](https://github.com/IntelRealSense/librealsense) - Intel Realsense library for D415/D435
* [Git LFS](https://git-lfs.github.com/) - Manage and store trained AI models
* [Travis CI](https://travis-ci.org/) - Perform build tests

## Authors

* **Michael Equi** - *Initial work*
* **Caelin Sutch** - *ROS2/phase 2 development*

See also the list of [contributors](https://github.com/JHS-ARCC-Club/jetson_car/graphs/contributors) who participated in this project.

## Acknowledgments

* README template source https://gist.github.com/PurpleBooth/109311bb0361f32d87a2

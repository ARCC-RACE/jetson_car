# jetson_car
Repository for the development of the JHS Autonomous Race Car Club's ROS  Jetson Car (TX2) with RealSense D400 series 3D camera

Check out system flowcharts: https://drive.google.com/open?id=1oScZmCizjCHx7lIW-BQ1HAR4U3CumyJK

Master   
[![Build Status](https://travis-ci.com/JHS-ARCC-Club/jetson_car.svg?branch=master)](https://travis-ci.com/JHS-ARCC-Club/jetson_car)

Development   
[![Build Status](https://travis-ci.com/JHS-ARCC-Club/jetson_car.svg?branch=development)](https://travis-ci.com/JHS-ARCC-Club/jetson_car)

## Getting Started
- Before setup make sure to `sudo apt-get update` and `sudo apt-get upgrade`
- If running on a Desktop computer without realsense packages installed run `catkin_make -DCATKIN_BLACKLIST_PACKAGES="realsense2_camera"` in workspace
- If you want to run the realsense2 ROS node on a desktop use the realsense2_camera pkg in the 	`drivers` folder by copying it into a local catkin_ws
- Download Xbox One Controller Driver https://github.com/paroj/xpad (Try controller without downloading this first)
- On the user interface ("remote" control machine) add `192.168.1.100 racecar` to the `/etc/hosts`
- When communicating with the jetson run `export ROS_MASTER_URI=http://racecar:11311`
- Install librealsense2 if interested (See steps below)

### Simulation Setup
- Go through Librealsense Setup (demo/viewing tools are not required)
- `cd ~/Desktop/jetson_car/jetsoncar_ws`
- `catkin_make` (even if this fails)
- `soruce devel/setup.bash`
- `rosdep install racecar_description`
- Install Tensor Flow https://www.tensorflow.org/install/ (download GPU version if you have one)
     - `sudo pip install tensorflow-gpu`
     - If running python program fails with error `ImportError: libcudnn.so.7: cannot open shared object file: No such file or directory` see https://stackoverflow.com/questions/41991101/importerror-libcudnn-when-running-a-tensorflow-program but make sure to download proper version of cuDNN (version 9.0) (https://developer.nvidia.com/rdp/cudnn-download)
- Install Keras `sudo pip install keras`
- `sudo pip install h5py`

### Librealsense Setup (D415/D435)
- Install the intel realsense 2 SDK (needed to catkin_make the catkin workspace if using the simulation of the physical sensor)
- https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
   - `sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE`
   - `sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u`
   - `sudo apt-get update`
   - `sudo apt-get install librealsense2`
   - `sudo apt-get install librealsense2-dev`
   - `sudo apt-get install librealsense2-dbg`
   - `sudo apt-get install librealsense2-dkms`  For demo
   - `sudo apt-get install librealsense2-utils` Tools for viewing physical camera data (See `/usr/bin/realsense-viewer`)
- When ready plug in the realsense camera and test it using the `/usr/bin/realsense-viewer`
   - If needed update the firmware using the instruction found here https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Linux-RealSense-D400-DFU-Guide.pdf


### Nvidia Jetson Setup without full setup script
- Follow Nvidia Jetson setup and run as user nvidia (password nvidia)
- Check to make sure you are running L4T version 28.2.1 for use with convenience scripts
   - If you version is off download jetpack here https://developer.nvidia.com/embedded/downloads#?search=jetpack%203.3
   - Follow this tutorial on reflashing the Jetson TX2 https://www.youtube.com/watch?v=D7lkth34rgM
- ~~clone https://github.com/jetsonhacks/installROSTX2.git to Desktop~~
- ~~Follow the directions to install this repository https://github.com/jetsonhacks/installRealSense2ROSTX (makes sure to install realsense before doing the kernal patches)~~
- clone https://github.com/JHS-ARCC-Club/jetson_car.git to Desktop
- Run these scripts in this order: installLibrealsense.sh, buildPatchedKernal.sh, installROS.sh
- Install USB driver for some arduino nanos https://devtalk.nvidia.com/default/topic/1032862/jetson-tx2/a-guide-to-solve-usb-serial-driver-problems-on-tx2/
- Run the `/scripts/jetson.sh` script to setup network
- Setup pip and download keras and tensorflow for python
   - `sudo apt-get install -y python-pip`
   - `pip install keras`
   - `sudo pip install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp33 tensorflow-gpu`
- `sudo usermod -a -G dialout $USER` to give proper permissions to the USB peripherals
- Add a udev rule for i2c-1 so that the IMU interface program can acces it
   - Go to `/etc/udev/rules.d`
   - Add a file called `50-i2c.rules`
   - Add the line `ACTION=="add", KERNEL=="i2c-[0-1]*", MODE="0666"` to the `50-i2c.rules` file
   - See https://forum.up-community.org/discussion/2141/tutorial-gpio-i2c-spi-access-without-root-permissions for additional information


### DualShock 4 Controller Setup (Ubuntnu 16.04)
- Connect to the controller by pressing and holding the `playstation button` and the `share` button until the light begins flashing white
- Go into Bluetooth settings and add the controller named `wireless controller`
- Test the connection by making sure the `/dev/input/js0` file exits
- Run the `joy_node` ROS node and echo the `/joy` topic to confirm data is being sent properly from the controller to the computer


### DualShock 4 Controller Setup (Ubuntu MATE RPI 3)
- Install `bluez` driver `sudo apt-get install bluez`
- https://github.com/macunixs/dualshock4-pi/blob/master/README.md
- Follow the steps for Ubutnu 16.04 above


### Prerequisites
- Realsense drivers installed on the jetson TX2
-

### Network Setup
- The `/etc/hosts` file should contain the following in addition to the default hostnames
      - `192.168.1.100   racecar`
- `export ROS_MASTER_URI=http://racecar:11311`
- `export ROS_IP=ip_of_comptuer`

#### Network Setup DEBUG

### Installing OS


##UPDATES NEEDED BELOW THIS POINT
--------------------------------

## Running the tests

Explain how to run the automated tests for this system (travis CI)

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why (coveralls)

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system (docker)

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://github.com/Michael-Equi/ROV_Test_Bench/blob/development/CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags).

## Authors

* **Michael Equi** - *Initial work*

See also the list of [contributors](https://github.com/Michael-Equi/ROV_Test_Bench/graphs/contributors) who participated in this project.

## Acknowledgments

* README Tempalte source https://gist.github.com/PurpleBooth/109311bb0361f32d87a2
* Inspiration
* etc

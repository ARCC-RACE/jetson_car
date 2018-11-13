# jetson_car
Repository for the development of the JHS Autonomous Race Car Club's ROS  Jetson Car (TX2) with RealSense D400


## Getting Started
- If you are on the jetson TX2 hardware run `catkin_make -DCATKIN_BLACKLIST_PACKAGES="realsense2_camera"` in jetsoncar_ws
- If you are on other hardware run `catkin_make -DCATKIN_BLACKLIST_PACKAGES="realsense2_jetson"` in jetsoncar_ws


### Simulation Setup
- Install the intel realsense 2 SDK (needed to catkin_make the catkin workspace)
   - https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
   - `sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCD` (Is this needed?)
   - `sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u` (Is this needed? Try without -u arg)
   - `sudo apt-get install librealsense2`
   - `sudo apt-get install librealsense2-dev`
   - `sudo apt-get install librealsense2-dbg`
 - `cd ~/Desktop/jetson_car/jetsoncar_ws`
 - `catkin_make` (even if this fails)
 - `soruce devel/setup.bash`
 - `rosdep install racecar_description`

### Nvidia Jetson Setup without full setup script
- Follow Nvidia Jetson setup and run as user nvidia (password nvidia)
- Check to make sure you are running L4T version 28.2.1 for use with convenience scripts
   - If you version is off download jetpack here https://developer.nvidia.com/embedded/downloads#?search=jetpack%203.3
   - Follow this tutorial on reflashing the Jetson TX2 https://www.youtube.com/watch?v=D7lkth34rgM
- ~~clone https://github.com/jetsonhacks/installROSTX2.git to Desktop~~
- ~~Follow the directions to install this repository https://github.com/jetsonhacks/installRealSense2ROSTX (makes sure to install realsense before doing the kernal patches)~~
- clone https://github.com/JHS-ARCC-Club/jetson_car.git to Desktop
- Run these scripts in this order: installLibrealsense.sh, buildPatchedKernal.sh, installROS.sh

### Prerequisites

### Network Setup

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

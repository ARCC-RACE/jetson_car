<<<<<<< HEAD
# Jetson Car
Repository for the development of the JHS Autonomous Race Car Club's ROS  Jetson Car (TX2) with RealSense D400.

The goal of this project is to develop preseason software technologies based on ROS. This project will explorer development paths and verify new technologies before more permanent hardware development. Key areas of testing are cameras, vector drive, controllers, and PID algortihms. Additionally new workflows/integration tactics are to be tested and documented (Travis CI, Docker, node documentation, doxygen). This project will expire at the beginning of the 2018-2019 robotics season.  

[![Build Status](https://travis-ci.com/Michael-Equi/ROV_Test_Bench.svg?branch=master)](https://travis-ci.com/Michael-Equi/ROV_Test_Bench)

[![Coverage Status](https://coveralls.io/repos/github/Michael-Equi/ROV_Test_Bench/badge.svg?branch=master)](https://coveralls.io/github/Michael-Equi/ROV_Test_Bench?branch=master)

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on the main ROV system (Master).

FOLLOW:
*  https://docs.google.com/document/d/1C32ucQTIAsE2H7u9OERmmCfyc6WzUvhrB2U6_GgZuWg/edit?usp=sharing

Initial SETUP:
* `cd ~/Desktop`
* `git clone https://github.com/Michael-Equi/ROV_Test_Bench.git`
* `git submodule update --init`
* `cd ROV_Test_Bench/scripts`
* `./GitSetup.sh`
* `./setup.sh`
* `sudo apt-get install ros-kinetic-joy`
* (only on rpi w/ ubuntu Mate) `sudo apt-get install samba`
* `sudo apt-get install python-smbus`
* `sudo apt-get install doxygen`
* `sudo apt-get install ros-kinetic-rosdoc-lite`
* `sudo apt-get install ros-kinetic-rosserial-arduino`
* `sudo apt-get install ros-kinetic-rosserial`
* Check individual package setup documentation
* `cd ~/Desktop/ROV_Test_Bench/ros_workspace`
* `catkin_make`
 * Check for errors
 * Check prerequisites
    
*Always run IDE's from terminal if on Ubuntu (just type the name of the IDE in terminal and click enter ex. clion)*

### Prerequisites

What things you should to install to develop and run software and how to install them

Code Blocks IDE on rpi ubuntu mate
* `sudo apt-get install codeblocks`

On the RPI turn on the CSI, SPI, I2C, and UART interfaces using `sudo raspi-config`

Setup the I2C interface on Ubutnu Mate
* `cd /boot/config.txt`
* uncomment `dtparam=i2c_arm=off` and change to `dtparam=i2c_arm=on`
* uncomment `dtparam=i2c_arm_baudrate=100000` and change to `dtparam=i2c_arm_baudrate=400000`
* restart pi

Setup the ros_lib file for arduino serial
* See sketchbook README.md documentation

RPI Camera node setup
* See raspicam_node README.md documentation

### Network Setup

What things you need to do so that the ROS network operates properly 

On ubuntu 16.04 go to Network Connections app and add a new ethernet connection (name the connection `ROVEthernetConnection`)
* On the topside computer have a static (manual) IP of `192.168.1.100`, netmask `24`, Gateway `92.168.1.1`, DNS server `27.0.1.1, 8.8.8.8, 192.168.1.1`
* On the bottomside computer have a static (manual) IP of `192.168.1.111`, netmask `24`, Gateway `192.168.1.1`, DNS server `127.0.1.1, 8.8.8.8, 192.168.1.1`
* Run the setupROSNetwork.sh script in the scripts folder

Once the network connection has been verified (on bottomside `ping master` / on topside `ping bottomside`)
* Run `sshSetup.sh` in the scripts folder
* Do not add any paraphrases 
* on bottomside `ssh master` / on topside `ssh bottomside`
* Make sure both work without entering a password 

#### Network Setup DEBUG
* IF you recieve `/usr/bin/ssh-copy-id: ERROR: ssh: connect to host bottomside port 22: Connection refused` go to the opposite machine from the one you recieved it on and run the following:
    * `sudo rm /etc/ssh/sshd_config`
    * `sudo apt-get purge openssh-server`
    * `sudo apt-get install openssh-server`
    * `./sshSetup.sh`
    
Other usefull links for common problems:
* https://superuser.com/questions/421004/how-to-fix-warning-about-ecdsa-host-key
* https://askubuntu.com/questions/762541/ubuntu-16-04-ssh-sign-and-send-pubkey-signing-failed-agent-refused-operation
* https://answers.ros.org/question/41446/a-is-not-in-your-ssh-known_hosts-file/

### Installing OS

A step by step series of examples that tell you how to get a development env running

On your Raspberry Pi 3 B make sure you are running ubuntu mate 16.04 (image here https://drive.google.com/open?id=1497jupJ2dBQqy_o_x5JBPTjY3lto7-rI)
* cat /etc/os-release

##UPDATES NEEDED BELOW THIS POINT
--------------------------------

=======
# jetson_car
Repository for the development of the JHS Autonomous Race Car Club's ROS  Jetson Car (TX2) with RealSense D400


## Getting Started

### Simulation Setup

### Nvidia Jetson Setup
- Follow Nvidia Jetson setup and run as user nvidia (password nvidia)
- Check to make sure you are running L4T version 28.2.1 for use with convenience scripts
 - If you version is off download jetpack here https://developer.nvidia.com/embedded/downloads#?search=jetpack%203.3
 - Follow this tutorial on reflashing the Jetson TX2 https://www.youtube.com/watch?v=D7lkth34rgM
 - 
- clone https://github.com/jetsonhacks/installROSTX2.git to Desktop
- Follow the directions to install this repository https://github.com/jetsonhacks/installRealSense2ROSTX
- clone https://github.com/JHS-ARCC-Club/jetson_car.git to Desktop
- `cd ~/Desktop`
- `installROSTX2/installROS.sh -p ros-kinetic-desktop`
- `installRealSenseROSTx2/installRealSenseROS.sh ~/Desktop/jetson_car/jetson_car_ws`

### Prerequisites

### Network Setup

#### Network Setup DEBUG

### Installing OS


##UPDATES NEEDED BELOW THIS POINT
--------------------------------
>>>>>>> 14341959d206be1a01360d8432d0112d80c58d13
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

# jetson_car
Repository for the development of the JHS Autonomous Race Car Club's ROS  Jetson Car (TX2) with RealSense D400

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on the main ROV system (Master).

### Simultation Setup:

### Nvidia Jetson TX2 Setup:
- Follow Nvidia Jetson setup and run as user nvidia (password nvidia)
- clone https://github.com/jetsonhacks/installROSTX2.git to Desktop
- Follow the directions to install this repository https://github.com/jetsonhacks/installRealSense2ROSTX
- clone https://github.com/JHS-ARCC-Club/jetson_car.git to Desktop
- `cd ~/Desktop`
- `installROSTX2/installROS.sh -p ros-kinetic-desktop`
- `installRealSenseROSTx2/installRealSenseROS.sh ~/Desktop/jetson_car/jetson_car_ws`

### Network Setup:

### Prerequisites (OUTDATED)

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

### Network Setup (OUTDATED)

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

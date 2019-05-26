#!/bin/bash
sudo pip install rospkg catkin_pkg

sudo apt-get install python-pyqt4

sudo apt-get install \
cmake gcc g++ qt4-qmake libqt4-dev \
libusb-dev libftdi-dev \
python-defusedxml python-vcstool \
libbluetooth-dev libspnav-dev \
pyqt4-dev-tools libcwiid-dev \
ros-kinetic-octomap-msgs        \
ros-kinetic-joy                 \
ros-kinetic-geodesy             \
ros-kinetic-octomap-ros         \
ros-kinetic-control-toolbox     \
ros-kinetic-pluginlib	       \
ros-kinetic-trajectory-msgs     \
ros-kinetic-control-msgs	       \
ros-kinetic-std-srvs 	       \
ros-kinetic-nodelet	       \
ros-kinetic-urdf		       \
ros-kinetic-rviz		       \
ros-kinetic-kdl-conversions     \
ros-kinetic-eigen-conversions   \
ros-kinetic-tf2-sensor-msgs     \
ros-kinetic-pcl-ros \
ros-kinetic-navigation \
ros-kinetic-ar-track-alvar-msgs

#Install Sophus
cd
git clone https://github.com/stonier/sophus -b release/0.9.1-kinetic
cd sophus
mkdir build
cd build
cmake ..
make
sudo make install
echo "## Sophus installed ##\n"

cd
git clone https://github.com/erlerobot/gym-gazebo
cd gym-gazebo
sudo pip install -e .

sudo pip install h5py
sudo apt-get install python-skimage

# install Theano
#cd ~/
#git clone git://github.com/Theano/Theano.git
#cd Theano/
#sudo python3 setup.py develop

# install tf for gpu
#pip install tensorflow-gpu

#install Keras
sudo pip install keras

# cd
# echo "export LD_LIBRARY_PATH=/usr/local/cuda-9.0/lib64:$LD_LIBRARY_PATH" >> bashrc
# echo "export PATH=/usr/local/cuda-9.0/bin:$PATH" >> ./bashrc

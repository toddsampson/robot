#!/bin/bash

cd ~/robot/src
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
cd depends
./download_debs_trusty.sh
sudo apt-get install -y build-essential cmake pkg-config
sudo dpkg -i debs/libusb*deb
sudo apt-get install -y libusb-1.0-0-dev
sudo apt-get install -y libturbojpeg libjpeg-turbo8-dev
sudo dpkg -i debs/libglfw3*deb
sudo apt-get install -f
sudo apt-get install -y libgl1-mesa-dri-lts-vivid
sudo apt-add-repository ppa:floe/beignet
sudo apt-get update
sudo apt-get install -y beignet-dev
sudo dpkg -i debs/ocl-icd*deb
# possibly install cuda for nvidia
sudo dpkg -i debs/{libva,i965}*deb
sudo apt-get install -f
sudo apt-get install -y libopenni2-devf
cd ..
mkdir build && cd build
cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
sudo cp ~/robot/src/navbot/udev_rules/58-kinect.rules /etc/udev/rules.d/
cd ~/robot/src
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/robot/src
catkin_make -DCMAKE_BUILD_TYPE="Release"

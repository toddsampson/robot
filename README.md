# NAVBOT

work by Todd Sampson and Michael Orr

TODO: Integrate old instructions

head_unit setup instructions

0. Plug-in the structure sensor and add the device to your virtual machine (if you are running one)
1. Download, bunzip2, and run the install script (as root) for OpenNI2 from: http://structure.io/openni
2. Run `sudo apt-get install ros-indigo-openni2*`
3. Reboot
4. Run `roslaunch openni2_launch openni2.launch` and then open rviz to ensure the depth camera data is publishing

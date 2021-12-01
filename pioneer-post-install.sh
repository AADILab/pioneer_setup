#!/usr/bin/env bash
# pioneer-post-install.sh
# Author: Connor Yates
# This script is meant to be run on the acer laptops used in the AADI lab 
# after a fresh install of Ubuntu Xenial 16.04. It downloads/installs ROS and the 
# Pioneer control code. After that, it sets up necessary networking, ssh, etc... config files.


# Check for internet connectivity first before trying the script, since it relies
# heavily on a working internet connection
if nc -zw1 google.com 443; then
	echo "Internet working. Begining setup..."
else
	echo "No internet connection detected."
	echo "Fix this issue and re-run this script."
	exit
fi
	
# Set up the files at the start, moving them from the current location to /tmp/
# so they don't clutter the system and this script can refer to their location explicitly
# The files are:
#    - hosts : networking config file /etc/hosts
#    - config : ssh config file for host/username auto-completion
#    - 99-usb-pioneer-hw.rules : udev rules for connecting the lidar and pioneer via USB
mv hosts config  99-usb-pioneer-hw.rules libaria_2.9.1+ubuntu16_amd64.deb /tmp/

# Check that computer is up to date
sudo apt-get update
sudo apt-get upgrade --yes
# NOTE: Check that "restricted", "universe", and "multiverse" are allowed repository sources (they are by default)

# Install pleasentries
sudo apt-get install wget curl vim git --yes

# ROS Kinetic installation commands
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full --yes
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential --yes

# Install ROS packages needed to run pioneer code
sudo apt-get install --yes \
    ros-kinetic-navigation\
    ros-kinetic-gmapping\
    ros-kinetic-ros-control\
    ros-kinetic-ros-controllers\
    ros-kinetic-rviz

# Set up catkin
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install ARIA (dependency of rosaria)
sudo dpkg -i /tmp/libaria_2.9.1+ubuntu16_amd64.deb

# Setup Pioneer code
cd ~/catkin_ws/src
git clone https://github.com/AADILab/rosaria.git
git clone https://github.com/robopeak/rplidar_ros.git
git clone https://github.com/AADILab/pioneer_2dnav.git
git clone https://github.com/AADILab/nav_bundle.git
git clone https://github.com/AADILab/simple_navigation_goals.git
git clone https://github.com/AADILab/pioneer_test.git
git clone https://github.com/AADILab/pioneer_description.git
git clone https://github.com/AADILab/aadi_networking.git

source ~/.bashrc
cd ~/catkin_ws/
catkin_make

# Set up USB aliasing
sudo mv /tmp/99-usb-pioneer-hw.rules /etc/udev/rules.d/
sudo usermod -a -G dialout $USER

# Edit the hosts file really quickly
# Comment out the other IP address related to this host
sed '/'"$HOSTNAME"'/ s/^/#/' -i /tmp/hosts
# Replace the first line with the host name
sed 's/aadiX/'"$HOSTNAME"'/' -i /tmp/hosts
sudo mv /tmp/hosts /etc/hosts

# Set up ssh
sudo apt-get install openssh-server --yes
mkdir -p ~/.ssh
mv /tmp/config ~/.ssh/

# Set up history search via up/down arrow keys
echo "bind '" >> ~/.bashrc
echo '"\e[A": history-search-backward' >> ~/.bashrc
echo "'" >> ~/.bashrc

echo "bind '" >> ~/.bashrc
echo '"\e[B": history-search-forward'  >> ~/.bashrc
echo "'" >> ~/.bashrc

# Clean up apt, since it will probably have unnecessary packages now
sudo apt autoremove --yes

echo "Installation complete. Edit the aadi.robots network configuration to reflect a manual IPV4 configuration. (See documentation for details)."
echo "Post-Installation script finished. Please restart the machine for changes to fully take effect."

#!/usr/bin/env bash
# pioneer-post-install.sh
# Author: Connor Yates
# Last edit: August 2017
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
#    - redis-3.0.conf : First config file for redis server 3.0
#    - redis-3.0.conf.local : Second config file for redis server 3.0
#    - 99-usb-pioneer-hw.rules : udev rules for connecting the lidar and pioneer via USB
mv hosts config redis-3.0.conf redis-3.0.conf.local 99-usb-pioneer-hw.rules /tmp/

# Check that computer is up to date
sudo apt-get update
sudo apt-get upgrade --yes
# NOTE: Check that "restricted", "universe", and "multiverse" are allowed repository sources (they are by default)

# Install pleasentries, like vim and git
sudo apt-get install vim git --yes

# ROS Kinetic installation commands
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full --yes
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential --yes

# Install ROS packages needed to run pioneer code
sudo apt-get install ros-kinetic-navigation ros-kinetic-gmapping ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-rviz ros-kinetic-rocon* --yes 

# Set up catkin
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install ARIA (dependency of rosaria)
wget -P /tmp/ http://robots.mobilerobots.com/ARIA/download/current/libaria_2.9.1+ubuntu16_amd64.deb
sudo dpkg -i /tmp/libaria_2.9.1+ubuntu16_amd64.deb

# Setup Pioneer code
cd ~/catkin_ws/src
git clone https://github.com/amor-ros-pkg/rosaria.git
git clone https://github.com/robopeak/rplidar_ros.git
git clone https://github.com/JenJenChung/pioneer_2dnav.git
git clone https://github.com/JenJenChung/nav_bundle.git
git clone https://github.com/JenJenChung/simple_navigation_goals.git
git clone https://github.com/JenJenChung/pioneer_test.git
git clone https://github.com/JenJenChung/pioneer_description.git
git clone https://github.com/JenJenChung/aadi_networking.git
git clone https://github.com/JenJenChung/rocon_test_cases.git

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

# Patch rocon hub via the configuration files
sudo mv /tmp/redis-3.0.conf /opt/ros/kinetic/share/rocon_hub/redis/redis-3.0.conf
sudo mv /tmp/redis-3.0.conf.local /opt/ros/kinetic/share/rocon_hub/redis/redis-3.0.conf.local

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

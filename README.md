ROS Navigation Stack
====================

A 2D navigation stack that takes in information from odometry, sensor
streams, and a goal pose and outputs safe velocity commands that are sent
to a mobile base.

 * AMD64 Debian Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__navigation__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__navigation__ubuntu_xenial_amd64__binary/)

Related stacks:

 * http://github.com/ros-planning/navigation_msgs (new in Jade+)
 * http://github.com/ros-planning/navigation_tutorials
 * http://github.com/ros-planning/navigation_experimental

For discussion, please check out the
https://groups.google.com/group/ros-sig-navigation mailing list.

## Install

* Initial catkin work space
```shell=
mkdir -p ~/navigation_ws/src
cd ~/navigation_ws
catkin_make
```
* Install require package
```
cd ~/navigation_ws/src
git clone https://github.com/oiz5201618/my_turtlebot
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
sudo apt-get -y install libbullet-dev libsdl-dev libsdl-image1.2-dev ros-kinetic-bfl ros-kinetic-move-base-msgs
```

#### Experimental

```shell=
cd ~/navigation_ws/src
git clone https://github.com/oiz5201618/navigation
cd ..
catkin_make
```

#### Original
```shell=
cd ~/navigation_ws/src
git clone https://github.com/oiz5201618/ros_navigation_orig
cd ..
catkin_make
```

## Usage (with Turtlebot3 simulation)

* Setup `~/.bashrc`
```shell=
export TURTLEBOT3_MODEL=burger
```
* Launch simulation world
```shell=
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
* [Download map file](https://drive.google.com/open?id=10fBV_JrsOpleqxqA4FI8h0AKa7CUeOw-) and configure it
```shell=
vi sim_map.yaml
```
* change this path to your home directory
```
image: $HOME/sim_map.pgm
```
* Launch navigation stack 
```shell=
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/sim_map.yaml
```
* Launch rviz
```shell=
rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_nav.rviz
```

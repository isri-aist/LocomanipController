# [LocomanipController](https://github.com/isri-aist/LocomanipController)
Humanoid loco-manipulation controller

[![CI](https://github.com/isri-aist/LocomanipController/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/LocomanipController/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/LocomanipController/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/LocomanipController)](https://github.com/isri-aist/LocomanipController/blob/master/LICENSE)

https://user-images.githubusercontent.com/6636600/209670856-4ebf4456-9cf0-483e-8f41-db9ce55c9795.mp4

## Features
- Completely open source! (controller framework: mc_rtc, simulator: Choreonoid, sample robot model: JVRC1)
- Accepts commands for trajectory and velocity of a loco-manipulation object.
- Automated management with CI: Dynamics simulation is run on CI to verify loco-manipulation.

## Technical details
This controller is a simple extension of [BaselineWalkingController](https://github.com/isri-aist/BaselineWalkingController) for loco-manipulation.
For more information on walking control, see [BaselineWalkingController](https://github.com/isri-aist/BaselineWalkingController).

## Install

### Requirements
- Compiler supporting C++17
- Tested with `Ubuntu 20.04 / ROS Noetic` and `Ubuntu 18.04 / ROS Melodic`

### Dependencies
This package depends on
- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc)
- [QpSolverCollection](https://github.com/isri-aist/QpSolverCollection)
- [NMPC](https://github.com/isri-aist/NMPC)
- [CentroidalControlCollection](https://github.com/isri-aist/CentroidalControlCollection)
- [BaselineWalkingController](https://github.com/isri-aist/BaselineWalkingController)
- [CnoidRosUtils](https://github.com/isri-aist/CnoidRosUtils) (Only for Choreonoid simulation)

### Preparation
1. (Skip if ROS is already installed.) Install ROS. See [here](http://wiki.ros.org/ROS/Installation) for details.
```bash
$ export ROS_DISTRO=melodic
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install ros-${ROS_DISTRO}-ros-base python-catkin-tools python-rosdep
```

2. (Skip if mc_rtc is already installed.) Install mc_rtc. See [here](https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/installation-guide.html) for details.
```bash
$ curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | sudo -E bash
$ sudo apt-get install libmc-rtc-dev mc-rtc-utils mc-state-observation jvrc-choreonoid libcnoid-dev ros-${ROS_DISTRO}-mc-rtc-plugin ros-${ROS_DISTRO}-mc-rtc-rviz-panel libeigen-qld-dev
```

### Controller installation
1. Setup catkin workspace.
```bash
$ mkdir -p ~/ros/ws_lmc/src
$ cd ~/ros/ws_lmc
$ wstool init src
$ wstool set -t src isri-aist/LocomanipController https://github.com/isri-aist/LocomanipController --git -y
$ wstool update -t src isri-aist/LocomanipController
$ wstool merge -t src src/isri-aist/LocomanipController/depends.rosinstall
$ wstool update -t src
```

2. Install dependent packages.
```bash
$ source /opt/ros/${ROS_DISTRO}/setup.bash
$ rosdep install -y -r --from-paths src --ignore-src
```

3. Build a package.
```bash
$ catkin build locomanip_controller -DCMAKE_BUILD_TYPE=RelWithDebInfo --catkin-make-args all tests
```

4. Setup controller
```bash
$ mkdir -p ~/.config/mc_rtc/controllers
$ cp ~/ros/ws_lmc/src/isri-aist/LocomanipController/etc/mc_rtc.yaml ~/.config/mc_rtc/mc_rtc.yaml
```

### Simulation execution
```bash
# Terminal 1
$ source ~/ros/ws_lmc/devel/setup.bash
$ roscore
# Terminal 2
$ source ~/ros/ws_lmc/devel/setup.bash
$ roscd locomanip_controller/cnoid/project/
$ /usr/share/hrpsys/samples/JVRC1/clear-omninames.sh
$ choreonoid LMC_JVRC1_Cart.cnoid --start-simulation
# Terminal 3
$ source ~/ros/ws_lmc/devel/setup.bash
$ roslaunch locomanip_controller display.launch
```

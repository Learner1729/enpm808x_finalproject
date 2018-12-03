# *Naivik: An Autonomous Mobile Robot*

*This repository is the part of ENPM808X: Software Development of Robotics ME Coursework.* 

[![Build Status](https://travis-ci.org/Learner1729/naivik_robot.svg?branch=master)](https://travis-ci.org/Learner1729/naivik_robot)
[![Coverage Status](https://coveralls.io/repos/github/Learner1729/naivik_robot/badge.svg?branch=master)](https://coveralls.io/github/Learner1729/naivik_robot?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/Learner1729/naivik_robot/blob/master/LICENSE)
---

## Table of Contents
- [Overview](#overview)
- [Author](#author)
- [SIP Process with Sprint Plannig](#sip_process)
- [Presentation](#present)
- [Class Diagram](#class)
- [Activity Diagram](#activity)
- [Directory hierarchy](#structure)
- [Prerequisites](#pre) 
- [Implementation](#implementation)
- [Documentation](#doc)
- [Reference Materials](#references)
- [TODO](#futurework)

## <a name="overview"></a> Overview 
This repository is created as the part of my project on building an autonomous mobile robot using simulated turtlebot platform for ACME Robotics (ENPM808x Final Project). The software implements autonomous navigation and mapping capability using ROS nodes and services and the simulated turtlebot platform.

An autonomous mobile robot have a capability to move from its current position to the goal position autonomously with the help of mapping and localizing algorithms. Such a robot performs tasks with a high degree of autonomy, which is particularly describe in fields such as spaceflight, household maintenance (such as cleaning), waste water treatment, indoor navigation and delivery goods and services. Example of such robots ranges from autonomous helicopters to Roomba, the robot vacuum cleaner.

## <a name="author"></a> Author

*Name:* Ashish Patel
*UID:* 115730625
*Course:* ENPM808X: Software Development in Robotics

## <a name="sip_process"></a> SIP Process with Sprint Planning
Sprint Planing is provided in the google doc file, click on the link to access it: [Sprint Planning](https://docs.google.com/document/d/1cnqYP7j8j8OXodav0haAKlPngPgLUw-r7h47pi6sqGg/edit?usp=sharing)

The SIP Process followed is detailed in a spreadsheet, click on the link to access it: [SIP](https://docs.google.com/spreadsheets/d/1nMbX9Id-yYUnSFjK3-XCJu1AtMnWx-2TkpUoiuIMqgM/edit?usp=sharing)

## <a name="present"></a> Presentation
The project presentation is made using google presentation slides, click on the link to access it: [Presentation](https://docs.google.com/presentation/d/1P-7ZiaSU_TAKtFjx63iGmTSdBFGq3v5k5EBiq1V8JrM/edit?usp=sharing)

## <a name="class"></a> Class Diagram
<p align="center">
<a target="_blank"><img src=""
alt="WorkUnderProgress" width="80" height="10" border="10" />
</a>
</p>

## <a name="activity"></a> Activity Diagram
<p align="center">
<a target="_blank"><img src=""
alt="WorkUnderProgress" width="80" height="10" border="10" />
</a>
</p>

## <a name="structure"></a> Directory hierarchy
``` 
catkin_ws/
-build/
-devel/
-src/
--<ROS package, this is your repsitory for commit>
---package.xml
---CMakeLists.txt
---include/
---src/
----tests/    -> Here is where unit testing will be implemented
---srv/
---tests/     -> Here is where rostest will be implemented
---rviz/
---world/
---UML/
----initial/
----final/
---results/
---launch/
---.gitignore
---.travis.yml
```
>Note: "-" indicates levels

## <a name="pre"></a> Prerequisites 

* [ROS Kinetic](https://wiki.ros.org/ROS/Installation) on Ubuntu 16.04. 
  >Note: Follow the installation steps given on ROS Kinetic installation page.

	After installing the full version of ROS kinetic distro we need to setup ROS by following the below steps. This steps are needed to be performed only once: <br/>
  **1.** Setting up rosdep systemwide: `$ sudo rosdep init` <br/>
	**2.** Setting up rosdep in a user account: `$ rosdep update` <br/>
	**3.** Setting up enivronment variables <br/>
  Updating the bashrc file: `$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc` <br/>
  Sourcing the bashrc file: `$ source ~/.bashrc` <br/>

* [Gazebo](http://gazebosim.org/) is a well-developed robust open source physic based simulator which can be used for simulating robotic systems. It can be used along with ROS as well as independently according to the user's need.

* [Rviz](http://wiki.ros.org/rviz) is specifically used for visualization of real & simulated robotic systems.

* [Turtlebot packages](http://wiki.ros.org/turtlebot_gazebo) 
  >Note: Gazebo & Rviz will be already installed as part of the ROS distro, however Turtlebot_Gazebo is need to be installed separately.

	To install turtlebot packages, open a terminal and run the following command: <br/>
  ```bash
  $ sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
  ```
  The developed package also depends on: *roscpp*, *geometry_msgs*, *sensor_msgs* *move_base_msgs*, *message_generation*, *image_transport*, *map-server*, *AMCL*, *gmapping*, *OpenCV* & *cv_bridge*

### <a name="workspace"></a> Creating a package workspace

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Before any update run the command `$ source devel/setup.bash` 

## <a name="implementation"></a> Implementation

### Standard install via command-line

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/Learner1729/naivik_robot.git
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH
```
>Note: The last command checks whether the environment variable includes the directory you are in or not. If it doesn't include please follow the above steps properly. This is the most important step to check that everything is installed and linked properly. 

## <a name="doc"></a> Documentation
WorkUnderProgress

## <a name="references"></a> Reference Materials
WorkUnderProgress

## <a name="futurework"></a> TODO
WorkUnderProgress
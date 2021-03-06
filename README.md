# *Naivik: An Autonomous Mobile Robot*

*This repository is the part of ENPM808X: Software Development of Robotics ME Coursework.* 

[![Build Status](https://travis-ci.org/Learner1729/naivik_robot.svg?branch=master)](https://travis-ci.org/Learner1729/naivik_robot)
[![Coverage Status](https://coveralls.io/repos/github/Learner1729/naivik_robot/badge.svg?branch=master)](https://coveralls.io/github/Learner1729/naivik_robot?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/Learner1729/naivik_robot/blob/master/LICENSE)
---

## Table of Contents
- [Overview](#overview)
- [Author](#author)
- [SIP Process with Sprint Planning](#sip_process)
- [Presentation](#present)
- [Class Diagram](#class)
- [Activity Diagram](#activity)
- [Directory hierarchy](#structure)
- [Prerequisites](#pre) 
- [Implementation](#implementation)
- [Documentation](#doc)
- [TODO](#futurework)

## <a name="overview"></a> Overview 
This repository is created as the part of my project on building an autonomous mobile robot using simulated turtlebot platform for ACME Robotics (ENPM808x Final Project). The software implements autonomous navigation and mapping capability using ROS nodes and services and the simulated turtlebot platform.

<p align="center">
  <img src="demo/v01D.gif" alt="v0.1D"/>
</p>

An autonomous mobile robot have a capability to move from its current position to the goal position autonomously with the help of mapping and localizing algorithms. Such a robot performs tasks with a high degree of autonomy, which is particularly describe in fields such as spaceflight, household maintenance (such as cleaning), waste water treatment, indoor navigation and delivery goods and services. Example of such robots ranges from autonomous helicopters to Roomba, the robot vacuum cleaner.

## <a name="author"></a> Author

My name is *Ashish Patel*, a developer and maintainer of this repository. I am a master student majoring in *Robotics at University of Maryland - College Park*. This repository is a part of the course ENPM808X - Software Development in Robotics. I hold a bachelors degree in Electronics & Communication from G H Patel College of Engineering located at Vallabh Vidyanagar, Gujarat, India.

## <a name="sip_process"></a> SIP Process with Sprint Planning
Sprint Planing is provided in the google doc file, click on the link to access it: [Sprint Planning](https://docs.google.com/document/d/1cnqYP7j8j8OXodav0haAKlPngPgLUw-r7h47pi6sqGg/edit?usp=sharing)

The SIP Process followed is detailed in a spreadsheet, click on the link to access it: [SIP](https://docs.google.com/spreadsheets/d/1nMbX9Id-yYUnSFjK3-XCJu1AtMnWx-2TkpUoiuIMqgM/edit?usp=sharing)

## <a name="present"></a> Presentation
The project presentation is made using google presentation slides, click on the link to access it: [Presentation](https://docs.google.com/presentation/d/1P-7ZiaSU_TAKtFjx63iGmTSdBFGq3v5k5EBiq1V8JrM/edit?usp=sharing)

## <a name="class"></a> Class Diagram
<p align="center">
<a target="_blank"><img src="UML/final/v0.1_class_diagram.png"
alt="WorkUnderProgress" width="1920" height="600" border="5" />
</a>
</p>

## <a name="activity"></a> Activity Diagram
<p align="center">
<a target="_blank"><img src="UML/final/v0.1_activity_diagram.png"
alt="WorkUnderProgress" width="1600" height="800" border="5" />
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
---srv/
---tests/
----launch/
---rviz/
---world/
---UML/
----initial/
----revised/
----final/
---results/
---docs/
---launch/
---.gitignore
---.travis.yml
```
>**Note:** "-" indicates levels

## <a name="pre"></a> Prerequisites
* [ROS Kinetic](https://wiki.ros.org/ROS/Installation) on Ubuntu 16.04. 
  >**Note:** Follow the installation steps given on ROS Kinetic installation page.

	After installing the full version of ROS kinetic distro we need to setup ROS by following the below steps. This steps are needed to be performed only once: <br/>
  **1.** Setting up rosdep systemwide: `$ sudo rosdep init` <br/>
	**2.** Setting up rosdep in a user account: `$ rosdep update` <br/>
	**3.** Setting up enivronment variables <br/>
  Updating the bashrc file: `$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc` <br/>
  Sourcing the bashrc file: `$ source ~/.bashrc` <br/>

* [Gazebo](http://gazebosim.org/) is a well-developed robust open source physic based simulator which can be used for simulating robotic systems. It can be used along with ROS as well as independently according to the user's need.

* [Rviz](http://wiki.ros.org/rviz) is specifically used for visualization of real & simulated robotic systems.

* [Turtlebot packages](http://wiki.ros.org/turtlebot_gazebo) 
  >**Note:** Gazebo & Rviz will be already installed as part of the ROS distro, however Turtlebot_Gazebo is need to be installed separately.

	To install turtlebot packages, open a terminal and run the following command: <br/>
  ```bash
  $ sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
  ```
  The developed package also depends on: *roscpp*, *geometry_msgs*, *sensor_msgs* *move_base_msgs*, *message_generation*, *image_transport*, *OpenCV* & *cv_bridge*

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
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/Learner1729/naivik_robot.git
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH
```
>**Note:** The last command checks whether the environment variable includes the directory you are in or not. If it doesn't include please follow the above steps properly. This is the most important step to check that everything is installed and linked properly. 

### Commands to run tests for generated package

Follow the below steps to compile and launching the tests:
```bash
$ cd ~/catkin_ws
$ catkin_make run_tests && catkin_test_results
```
>**Note:** In the above command, firstly the test binary will get compiled and then first unit test will be run and then rostest will run to test ROS interface. ROSTEST uses launch file to run the code. Test launch file are in tests/launch/ directory


### Building for code coverage

Follow the below steps to generate the code coverage report on your local computer <br/>
**1.** Install lcov using the command `$ sudo apt-get install -y lcov` <br/>
**2.** Open a new terminal and run the below command `cd ~/catkin_ws && catkin_make run_tests && catkin_tests_results` <br/>
**3.** Generate the code coverage report `$ lcov --directory . --capture --output-file coverage.info` <br/>
**4.** Filter out the system and test code data including user-defined header files `$ lcov --remove coverage.info '/usr/*' '/opt/*' '*/devel/*' 'tests/*' --output-file coverage.info` <br/>
**5.** Generate an HTML report from a captured coverage information and store it in a *output* directory in a same directory `$ genhtml coverage.info --output-directory output` <br/>

>**Note:** Once it is done you can go through the *output* folder and open *index.html* to check coverage percentile. It will display both Line and Function coverage as shown in *coverage_report* image in */results* directory.

### Run Naivik Robot using rosrun

After following the above standard install commands, follow the below commands to run turtlebot_gazebo and default world

```bash
$ roslaunch turtlebot_gazebo turtlebot_world.launch
```
You will see a gazebo window open along with default turtlebot_world with obstacles as well as the turtlebot robot. Now, we will run the walker node. Open a new shell and follow the below commands.

```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun naivik_robot naivik_robot_node
```
Now, the turtlebot should start moving forward. As the robot drives and when it encounters an obstacle, it will stop, turn in place to free itself and then continue driving.

>**Note:** We can add obstacles when the simulation is going on, which can be useful to check whether our code is working properly or not.

### Run Naivik Robot using launch file

The ROS node and turtlebot_gazebo environment can be start using a single launch file as follows:

```bash
$ roslaunch naivik_robot WalkerAlgorithm.launch
```
>**Note:** No need to start roscore, roslaunch will automatically start if it detects that it is not already running.

To launch Naivik Robot, custom gazebo world and rviz altogether use the below command.
```bash
$ roslaunch naivik_robot WalkerAlgorithm_CustomWorld.launch
```

### Run changeThresholdService

There is rosservice which is used to change the distance threshold at which robot will stop moving to avoid an obstacle. As the vehicle moves throughout the environment, it checks its laser scan data for any obstacles closer than this distance threshold. To call this service, open a new terminal and follow the below steps. 

```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosservice call /changeThresholdService "changeThreshold: 0.5"
```

### Run changeLinearSpeedService

You may want to change the speed at which the robot moves forward. To do so, you can issue another rosservice call to the robot. The robot will see this service and change the linear speed to the value passed in to the service. To make this service call, open a new terminal and follow the below steps. 

```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosservice call /changeLinearSpeedService "changeLSpeed: 1.0"
```

### Run changeAngularSpeedService

You may want to change the speed at which the robot moves. To do so, you can issue another rosservice call to the robot. The robot will see this service and change the angular speed to the value passed in to the service. To make this service call, open a new terminal and follow the below steps.

```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosservice call /changeAngularSpeedService "changeASpeed: 1.0"
```

### Run controlMotionService

Using this ROS service, you can stop the robot in place (and then later start its motion). To make this service call, open a new terminal and follow the below steps.

```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosservice call /controlMotionService "motion: true"
```
>**Note:** The value of motion will be *true* if you want to stop the robot, and to resume the motion back call the same service and change the value to *false*.

### Run takeImageService

When the vehicle is moving, and if the user want to take a picture. You can issue a rosservice call to the robot. The robot will see this service and change the *takeImageFlag_* flag so that next time it sees the `/camera/rgb/image_raw topic`, it will take and save an image. To make this service call, open a new terminal and follow the below steps.

```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosservice call /takeImageService "request: true"
```

## <a name="doc"></a> Documentation
Generating documentation using doxygen <br/>
**1.** To install *doxygen* run the following command: `$ sudo apt-get install doxygen` <br/>
**2.** Generate doxygen config file into your cloned github repository. `$ doxygen -g <config_file>` <br/>
**3.** The generated config_file includes the parameter through which doxygen generates your documentation. You need to modify this by referring to this [link.](https://www.ibm.com/developerworks/aix/library/au-learningdoxygen/index.html) <br/>
**4.** Run the following command: `$ doxygen <config_file>` <br/>
**5.** Doxygen files will be generated to *html* and *latex* folder to your output location specified in config_file <br/>
**6.** To view them in a browser go to the output folder and run the commands below:
```bash 
$ cd html
$ firefox index.html
```
>**Note:** If the above steps doesn't work for you, you can refer the documentation in /docs folder of the repository. I have also provided the config file for your reference

Click the link to see the generated document for version 0.1: [DOXYGEN Document](http://htmlpreview.github.io/?https://github.com/Learner1729/naivik_robot/blob/master/docs/html/index.html)

>**Note:** The main page doesn't print any information, it's because I havn't included README.md file while generating docs files.

## <a name="futurework"></a> TODO

This is v0.1 which implements a random motion of robot in an environment. It also includes ROS API to change parameters like linear velocity, angular velocity, capture image, stop/resume motion and change distance threshold value. Now, the next version will build a map which can be used in ROS navigation stack to make the robot travel from one location to another based on USER commands.

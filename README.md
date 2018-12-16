# *Naivik: An Autonomous Mobile Robot v0.2*

*This repository is the part of ENPM808X: Software Development of Robotics ME Coursework.* 

[![Build Status](https://travis-ci.org/Learner1729/naivik_robot.svg?branch=v0.2)](https://travis-ci.org/Learner1729/naivik_robot)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/Learner1729/naivik_robot/blob/master/LICENSE)
---

## Table of Contents
- [Overview](#overview)
- [Author](#author)
- [SIP Process with Sprint Planning](#sip_process)
- [Presentation](#present)
- [Class Diagram](#class)
- [Activity Diagram](#activity)
- [Prerequisites](#pre) 
- [Implementation](#implementation)
- [Documentation](#doc)
- [Demo](#demo)
- [TODO](#futurework)

## <a name="overview"></a> Overview 
This repository is created as the part of my project on building an autonomous mobile robot using simulated turtlebot platform for ACME Robotics (ENPM808x Final Project). The software implements autonomous navigation and mapping capability using ROS nodes and services and the simulated turtlebot platform.

An autonomous mobile robot have a capability to move from its current position to the goal position autonomously with the help of mapping and localizing algorithms. Such a robot performs tasks with a high degree of autonomy, which is particularly describe in fields such as spaceflight, household maintenance (such as cleaning), waste water treatment, indoor navigation and delivery goods and services. Example of such robots ranges from autonomous helicopters to Roomba, the robot vacuum cleaner.

## <a name="author"></a> Author

My name is *Ashish Patel*, a developer and maintainer of this repository. I am a master student majoring in *Robotics at University of Maryland - College Park*. This repository is a part of the course ENPM808X - Software Development in Robotics. I hold a bachelors degree in Electronics & Communication from G H Patel College of Engineering located at Vallabh Vidyanagar, Gujarat, India.

## <a name="sip_process"></a> SIP Process with Sprint Planning
Sprint Planing is provided in the google doc file, click on the link to access it: [Sprint Planning](https://docs.google.com/document/d/1cnqYP7j8j8OXodav0haAKlPngPgLUw-r7h47pi6sqGg/edit?usp=sharing)

The SIP Process followed is detailed in a spreadsheet, click on the link to access it: [SIP](https://docs.google.com/spreadsheets/d/1nMbX9Id-yYUnSFjK3-XCJu1AtMnWx-2TkpUoiuIMqgM/edit?usp=sharing)

## <a name="present"></a> Presentation
The project presentation is made using google presentation slides, click on the link to access it: [Presentation](https://docs.google.com/presentation/d/1P-7ZiaSU_TAKtFjx63iGmTSdBFGq3v5k5EBiq1V8JrM/edit?usp=sharing)

## <a name="pre"></a> Prerequisites

Apart from v0.1 prerequisites specified in master branch below are the additional dependency needed for implementing autonomous navigation.

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
$ cd naivilk_robot
$ git branch -a
$ git checkout v0.2
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH
```
>**Note:** The last command checks whether the environment variable includes the directory you are in or not. If it doesn't include please follow the above steps properly. This is the most important step to check that everything is installed and linked properly. 

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

Click the link to see the generated document for version 0.2: [DOXYGEN Document]()

## <a name="demo"></a> Demo

## <a name="futurework"></a> TODO


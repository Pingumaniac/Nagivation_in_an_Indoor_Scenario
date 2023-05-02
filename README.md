# Nagivation_in_an_Indoor_Scenario
CS 5891 Special Topics - The Algorithms of Robotics Final Project

## About Members 

#### Young-jae Moon
* M.Sc. in computer science and Engineering Graduate Fellowship recipient at Vanderbilt University (January 2023 - December 2024).
* Email: youngjae.moon@Vanderbilt.Edu

#### Rubin Zou
* B.Sc. in computer science at Vanderbilt University
* Email: rubin.h.zou@vanderbilt.edu

## Advisor

#### Professor Jie Wing Wu (she/her)
* Assistant Professor at Vanderbilt University
* Email: jiewing.yu@vanderbilt.edu

#### Hao (Simon) Yang (he/him)
* Ph.D. in computer science at Vanderbilt University
* Email: hao.yang@vanderbilt.edu

## Instructions for launching the simulation of the robot

1. Follow the instructions for setting up Turtlebot 3 for ROS Melodic as shown in the (Section 6.1 and 7.1 of the Tutorial) - https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation 
2. After downloading this GitHub repository, unzip it.
3. Open the terminal and move to the corresponding workspace. For example,
```
cd catkin_ws/src
```

4. Then, source the setup.bash file.
```
source devel/setup.bash
```

5. We can follow the Tutorial to launch the TurtleBot 3 House map with the robot.
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

## Instructions for navigating the robot
Launch the gmapping package to perform SLAM:
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
Launch the navigation stack:
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

Run the modified Python node:
```
rosrun turtlebot3_slam_navigation_node turtlebot3_slam_navigation_node.py
```

Enter the starting and target room names when prompted, and observe the robot navigating between the rooms and returning to the starting room.

This completes the navigation part of the home service mission using Python for the Turtleot 3 Waffle Pi in the "house" world.

## Instructions for running the SLAM

```
source devel/setup.bash
```
```
export TURTLEBOT3_MODEL=burger
```
```
roslaunch turtlebot_navigation_and_mapping explore.launch
```

# ASSIGNMENT OF TONY HO
This package is used for ROS Melodic in Ubuntu18.04

## I. Setup Environment
### 1. Install Dependences
```
sudo apt-get install ros-$ROS_DISTRO-universal-robots
sudo apt-get install ros-melodic-tf*
```
### 2. Build the ur5-controller package
1. Clone package into src of your workspace
```
cd ~/catkin_ws/src
git clone https://github.com/tonyhoVN/UR5-Controller.git
```
2. Build package by catkin
```
cd ~/catkin_ws
catkin_make --pkg assignment_ur && catkin_make install --pkg assignment_ur
source devel/setup.bash && source install/setup.bash
```

## II. Solving Problems 

### Problem 1: Sine wave signal
Joints of robot move as sine wave function around its home position (vertical position). The input signal has amplitude of 30 degrees and angular velocity is 90 degree/second  
1. Launch simulation environment
```
roslaunch assignment_ur assignment1.launch
```
2. Start simulation: go to gazebo gui and press the "play" button to see the result

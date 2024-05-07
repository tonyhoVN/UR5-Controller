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

### Problem2: Joint Space and Cartesian Space Trajectory 
The trajectory in this problem is generated as trapezoidal profile. For simple execution, it is assumed all joints share the same velocity and acceleration. The time step between points in trajectory is 0.2 seconds

0. Additional service: move robot to home position
    * You can move robot to home position by command
    ```
    rosservice call /ur5_custom_service/move_home "{}"
    ```

1. Joint space motion: perform trapezoidal motion between 2 point in joint space 
    * Input message will have form:    
        * joints1: start position (rad)  
        * joints2: final position (rad)
        * max_vel: maximum joint velocity (rad/s)
        * maximum joint acceleration (rad/s^2)
    * if joints1 is empty, start will be set as current robot state 
    * Example:
        ```
        rosservice call /ur5_custom_service/move_joint_space "{'joints1': [], 'joints2': [0, 0, 0, 0, 0, 0], 'joints_vel': 0.2, 'joints_acc': 0.2}"
        ```


2. Cartesian space motion: perform linear trapezoidal motion between 2 point in task space (assume there is no change in rotation) -> Use IK solver (KDL) to convert to joint space
- start position (m)
- final position (m)
- maximum linear velocity (m/s)
- maximum linear acceleration (m/s^2)

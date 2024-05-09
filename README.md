# ASSIGNMENT OF TONY HO
This package is used for ROS Melodic in Ubuntu18.04

## I. Setup Environment
1. Clone package into src of your workspace
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/tonyhoVN/UR5-Controller.git
    ```
2. Build package by catkin
    ```
    cd ~/catkin_ws
    catkin_make --pkg assignment_ur && source devel/setup.bash
    ```

## II. Solving Problems 

### Problem 1: Sine wave signal
- Joints of robot move as sine wave function around its home position (vertical position). The input signal has amplitude of 30 degrees and angular velocity is 90 degree/second  
- Execution process:
    - Step1: Launch simulation environment
    ```
    roslaunch assignment_ur assignment1.launch
    ```
    - Step2: Start simulation: go to gazebo gui and press the "play" button to see the result

### Problem2: Joint Space and Cartesian Space Trajectory 
- The trajectory in this problem is generated as trapezoidal profile. For simple execution, it is assumed all joints share the same velocity and acceleration. The time step between points in trajectory is 0.2 seconds. 
- Execution process:
    - Step1: Launch simulation environment
    ```
    roslaunch assignment_ur assignment2.launch
    ```
    - Step2: go to gazebo gui and press the "play" button to activate simulation
    - Step3: Call the following services to make movements

0. Additional service: move robot to home position to advoid singularity position 
    * You can move robot to home position by command
    ```
    rosservice call /ur5_custom_service/move_home "{}"
    ```

1. Joint space motion: perform trapezoidal motion between 2 point in joint space 
    * Input message will have form:    
        * joints1: start position (deg)  
        * joints2: final position (deg)
        * joint_vel: maximum joint velocity (deg/s)
        * joint_acc: maximum joint acceleration (deg/s^2)
    * if joints1 is empty, start will be set as current robot state 
    * Example:
        ```
        rosservice call /ur5_custom_service/move_joint_space "{'joints1': [], 'joints2': [0, 0, 70, 30, 90, 0], 'joints_vel': 30, 'joints_acc': 100}"
        ```

2. Cartesian space motion: perform linear trapezoidal motion between 2 point in task space (assume there is no change in rotation). After cartesian trapezoidal trajectory is calculated, IK solver (KDL) is used to convert point in cartestian to joint space. Call the move_home service firstly to avoid singularity.
    * Input message will have form:    
        * point1: start position (mm)  
        * point2: final position (mm)
        * linear_vel: maximum linear velocity (mm/s)
        * linear_acc: maximum linear acceleration (mm/s^2)
    * If point1 is empty, start will be set as current robot state 
    * Example:
        ```
        rosservice call /ur5_custom_service/move_cartesian_space "{'point1': [], 'point2': [400, 100, 100], 'linear_vel': 50, 'linear_acc': 100}"
        ```

### Problem3: Python API 
- The API includes python functions calling services in the previous problems. You can check all functions in folder UR5_API. 
- The example of usage is give in script/example_api.py. Use rosrun to execute this file after launch assignment2.launch. 
    ```
    rosrun assignment_ur example_api.py
    ```

### Problem4: Integrate LLM into system 
- This session proposed method to use LLM model to callback API functions in previous session with input commands from users. The LLM uses the pre-defined prompt to generate python functions directly from user commands without retrain the model. The architecture of system is shown in following figure.

- Method: put the given prompt in to any LLM model (ChatGPT, Gemini, Co-Pilot) to make it become custom code generator from text commands. 
    ```
    You are a tool to convert user commands into given coding functions for controlling robotics manipulator. 

    The response will be only one of following funtions without display any additional information:
    1. move_home(): move robot to home position 
    2. get_robot_status(): get the current position of joints, joint names, and position of end_effector w.r.t  base_link
    2. move_joint_space(joint1=[], joint2=[], max_joint_velocity=0, max_acceleration=0): move robot in joint space from position1 to position2 with maximum joint velocity and accleration
    3. move_cartesian_space(point1=[], point2=[], max_joint_velocity=0, max_acceleration=0): move robot in cartesian space from point1 to point2 with maximum linear velocity and accleration

    The command with start from the next input.
    ```
- This picture shows the example of using such prompt in ChatGPT model

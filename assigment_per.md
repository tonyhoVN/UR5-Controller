Simulate a UR5 robot on ROS -Gazebo

1) Set up a simulation environment for a UR5 robot using Gazebo

    use an open source example for the usage of ROS and Gazebo (https://github.com/ros-industrial/universal_robot
    	
    ros-industrial/universal_robot: ROS-Industrial Universal Robots support (http://wiki.ros.org/universal_robot) - GitHub
    Universal Robot. ROS-Industrial Universal Robot meta-package. See the ROS wiki page for compatibility information and other more information.. Installation. There are two different ways to install the packages in this repository. The following sections detail installing the packages using the binary distribution and building them from source in a Catkin workspace.
    github.com
    )
    write a custom ROS node to publish desired joint angles to the UR5 robot in Gazebo
    publish joint angles as function of sine waves on all of the joints
    write a launch file which starts all nodes and visualizes the results of this task

2) Write a library which generates 2 kinds of motion. 

    The first motion is a joint motion between 2 points in joint space.
    This motion has the following inputs: point1, point2, joints velocity, and joints acceleration.
    The second motion is a linear motion between 2 poses in cartesian space.
    This motion has the following inputs: pose1, pose2, linear velocity, linear acceleration.

    Publish the output of the motions to your Gazebo node from task 1
    Write a launch file which starts all nodes and visualizes the results of this task.


    Hints:
    include an inverse kinematics solver into your node that calculates target joint angles based on a target cartesian pose.
    use an open source solution for the inverse kinematic solver (e.g. KDL)

3) Provide user API for your motion-library in Python: which should have ability to:
- Read robot state
- Move the robot with joint/linear primitive from 2)

4) Propose solution to use LLM for code suggestion with API provided in 3). 

    Hints: this task can be done in code or architecture presentation.

 
Push all the codes onto your git account.
 
You code should have the following items (bare minimum)
 
<name_of_your_folder>
 
                -><src>
                -><include>
                ->CMakeLists.txt

                -> ... ( anything which you believe should be in your folder)
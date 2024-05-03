# ASSIGNMENT OF TONY

Running command: "make -j8 -l8" in "/home/tony/neura_ws/build"

## 1. Install UR industrial ros
```
sudo apt-get install ros-$ROS_DISTRO-universal-robots
```

## 2. Build the package

1. Clone package into src of your workspace
2. Build package

```
catkin_make --pkg assignment_ur
source devel/setup.bash
```

## 3. Launch the simulation 

1. Run the launch file 
```
roslaunch assignment_ur system.launch
```
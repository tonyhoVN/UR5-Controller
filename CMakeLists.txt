cmake_minimum_required(VERSION 3.0.2)
project(assignment_ur)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    control_msgs
    sensor_msgs
    trajectory_msgs
    message_generation
    actionlib
    orocos_kdl
    kdl_parser
    tf
)

# find_package(orocos_kdl REQUIRED)
catkin_python_setup()

## ROS Service
add_service_files(
  FILES
  GetState.srv
  JointSpaceMotion.srv
  CartesianSpaceMotion.srv
  MoveHome.srv
)

## ROS Message
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## Add custom library
add_library(trajectory_generator
  src/trajectory_generator.cpp
)
target_link_libraries(trajectory_generator
  ${catkin_LIBRARIES}
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_generator
  CATKIN_DEPENDS message_runtime
  DEPENDS orocos_kdl
)

## Include library
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Build   ##
#############
add_executable(UR5_service src/UR5_service.cpp)
target_link_libraries(UR5_service 
  ${catkin_LIBRARIES}
  trajectory_generator
)
add_dependencies(UR5_service assignment_ur_gencpp)

add_executable(test_lib src/test_lib.cpp)
target_link_libraries(test_lib 
  ${catkin_LIBRARIES} 
  trajectory_generator
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS scripts/example_api.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS 
  UR5_API/UR5_API.py  
  UR5_API/__init__.py  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Additional files 
install(DIRECTORY UR5_API
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/UR5_API
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# install(DIRECTORY
#   urdf 
#   config 
#   launch
#   srv
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




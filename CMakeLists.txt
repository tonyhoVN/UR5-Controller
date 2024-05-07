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
)
find_package(orocos_kdl REQUIRED)

## ROS Service
add_service_files(
  FILES
  GetState.srv
  JointSpaceMotion.srv
  CartesianSpaceMotion.srv
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
  ${orocos_kdl_INCLUDE_DIRS}
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
  ${orocos_kdl_INCLUDE_DIRS}
)

#############
## Build   ##
#############
add_executable(UR5_service src/UR5_service.cpp)
target_link_libraries(UR5_service ${catkin_LIBRARIES})
add_dependencies(UR5_service assignment_ur_gencpp)

add_executable(test_lib src/test_lib.cpp)
target_link_libraries(test_lib 
  ${catkin_LIBRARIES} 
  trajectory_generator
  ${orocos_kdl_LIBRARIES}
)

#############
## Install ##
#############

# Install Trajectory_generator library
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(TARGETS trajectory_generator 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


# Python files
catkin_install_python(PROGRAMS scripts/assignment1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/test1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/UR5_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Additional files 
install(DIRECTORY 
  UR5_API
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  urdf 
  config 
  launch
  srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




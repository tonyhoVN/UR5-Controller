cmake_minimum_required(VERSION 3.0.2)
project(assignment_ur)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    control_msgs
    sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assignment
)

#############
## Install ##
#############

## Mark executables for installation
# install(TARGETS wave_cmd
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(PROGRAMS scripts/assignment1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/test1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  config 
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


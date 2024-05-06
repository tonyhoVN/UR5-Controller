cmake_minimum_required(VERSION 3.0.2)
project(assignment_ur)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    control_msgs
    sensor_msgs
    message_generation
)

add_service_files(
  FILES
  GetState.srv
  JointSpaceMotion.srv
  CartesianSpaceMotion.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assignment
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

### Build #####
add_executable(UR5_service src/UR5_service.cpp)
target_link_libraries(UR5_service ${catkin_LIBRARIES})
add_dependencies(UR5_service assignment_ur_gencpp)

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

catkin_install_python(PROGRAMS scripts/UR5_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  UR5_API
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  urdf 
  config 
  launch
  srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


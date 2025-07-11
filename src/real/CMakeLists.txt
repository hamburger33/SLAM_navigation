cmake_minimum_required(VERSION 3.0.2)
project(real)

# 设置C++17标准
add_compile_options(-std=c++17)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## CAN数据接收节点
add_executable(can_data_receiver_node 
  node/can_data_receiver_node.cpp
  src/can_data_receiver.cpp
)

target_link_libraries(can_data_receiver_node
  ${catkin_LIBRARIES}
)

## 里程计计算节点
add_executable(wheel_odom_calculator_node 
  node/wheel_odom_calculator_node.cpp
  src/wheel_odom_calculator.cpp
)

target_link_libraries(wheel_odom_calculator_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Install executables
install(TARGETS 
  can_data_receiver_node
  wheel_odom_calculator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install script files
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
)

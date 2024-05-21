cmake_minimum_required(VERSION 3.0.2)
project(rldp5_scripts)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

# catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## target_link_libraries(image ${catkin_LIBRARIES})


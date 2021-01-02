cmake_minimum_required(VERSION 3.0.2)
project(offboard)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  roslaunch
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES offboard
   CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

add_executable(setpoint src/setpoint_node.cpp)
add_executable(logs src/logs_node.cpp)
# add_executable(hovering src/hovering_node.cpp)
add_executable(local src/local_node.cpp)
# add_executable(gps src/gps_node.cpp)
add_executable(setmode_offb src/setmode_offb.cpp)

target_link_libraries(setpoint
  ${catkin_LIBRARIES}
)
target_link_libraries(logs
  ${catkin_LIBRARIES}
)
# target_link_libraries(hovering
#   ${catkin_LIBRARIES}
# )
target_link_libraries(local
  ${catkin_LIBRARIES}
)
# target_link_libraries(gps
#   ${catkin_LIBRARIES}
# )
target_link_libraries(setmode_offb
  ${catkin_LIBRARIES}
)